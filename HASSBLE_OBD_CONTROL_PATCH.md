# HassBle OBD control-sequence patch

Target: `eigger/hassble-android` v1.7.0

## Problem

OBD sensor polling sends ELM327 ASCII commands terminated with `\r`, waits for the `>`
prompt/response, and serializes access through `deviceMutexes`.

OBD controls currently call `Elm327Source.write(device, hex)`, which hex-decodes the
configured string and writes raw bytes directly to the BLE TX characteristic. That bypasses
the ELM command parser and cannot safely execute module-header/session/action/restore
sequences.

For Nissan BCM controls we need an atomic sequence such as:

```text
ATSH745
10C0
30172001
1081
ATSH7DF
```

Each command must be sent as ASCII + CR and the ELM prompt must be received before the
next command is sent.

## Proposed schema

Extend `ControlConfig` in
`app/src/main/java/dev/eigger/hassble/config/Config.kt`:

```kotlin
@Serializable
data class ControlConfig(
    val key: String,
    val type: ControlType,
    val name: String? = null,
    val icon: String? = null,
    @SerialName("entity_category") val entityCategory: String? = null,
    val action: ControlAction? = null,
    val command: Map<String, String> = emptyMap(),
    @SerialName("pre_commands") val preCommands: List<String> = emptyList(),
    @SerialName("post_commands") val postCommands: List<String> = emptyList(),
    val options: List<String> = emptyList(),
    val min: Double? = null,
    val max: Double? = null,
    val step: Double? = null,
)
```

For `source: obd`, command values are ELM command strings, not BLE payload hex.
`gatt_notify` retains the current raw-hex behavior.

## Elm327Source API

In `AcquisitionSources.kt`, add:

```kotlin
suspend fun executeCommands(
    device: DeviceConfig,
    commands: List<String>,
): List<String?>
```

Keep `write()` for compatibility if desired, but OBD controls should use
`executeCommands()`.

## NordicElm327Source

Refactor the existing `sendCommand()` so the mutex can cover an entire sequence.

```kotlin
private suspend fun sendCommand(
    deviceId: String,
    txChar: ClientBleGattCharacteristic,
    cmd: String,
): String? {
    val mutex = deviceMutexes.getOrPut(deviceId) { Mutex() }
    return mutex.withLock {
        sendCommandLocked(deviceId, txChar, cmd)
    }
}

private suspend fun sendCommandLocked(
    deviceId: String,
    txChar: ClientBleGattCharacteristic,
    cmd: String,
): String? {
    rxBuffers[deviceId]?.let { synchronized(it) { it.setLength(0) } }

    val deferred = CompletableDeferred<String>()
    pendingDeferreds[deviceId] = deferred
    val payload = (cmd + "\r").toByteArray(Charsets.US_ASCII)

    val timeoutMs = if (cmd.trim().uppercase().startsWith("AT")) {
        SINGLE_FRAME_TIMEOUT_MS
    } else {
        MULTIFRAME_TIMEOUT_MS
    }

    try {
        txChar.write(DataByteArray(payload), BleWriteType.NO_RESPONSE)
    } catch (e: CancellationException) {
        pendingDeferreds.remove(deviceId)
        throw e
    } catch (e: Exception) {
        pendingDeferreds.remove(deviceId)
        throw IOException("BLE write failed: ${e.message}", e)
    }

    val resp = withTimeoutOrNull(timeoutMs) { deferred.await() }
    pendingDeferreds.remove(deviceId)
    return resp
}
```

Then implement:

```kotlin
@SuppressLint("MissingPermission")
override suspend fun executeCommands(
    device: DeviceConfig,
    commands: List<String>,
): List<String?> {
    if (commands.isEmpty()) return emptyList()

    val client = activeConnections[device.id] ?: return emptyList()
    val obd = device.obd ?: return emptyList()

    val services = client.discoverServices()
    val service = services.findService(uuidFrom(obd.serviceUuid)) ?: return emptyList()
    val txChar = service.findCharacteristic(uuidFrom(obd.txCharUuid)) ?: return emptyList()

    val txDelayMs = parseDurationMs(obd.txDelay, 50L)
    val mutex = deviceMutexes.getOrPut(device.id) { Mutex() }

    return mutex.withLock {
        buildList {
            for ((index, cmd) in commands.withIndex()) {
                LiveEventLogger.log(
                    LogType.TX,
                    "device=${device.id}, control cmd=$cmd",
                )

                val resp = sendCommandLocked(device.id, txChar, cmd)
                add(resp)

                LiveEventLogger.log(
                    LogType.RX,
                    "device=${device.id}, control cmd=$cmd -> " +
                        (resp?.trim()?.replace(Regex("[\\r\\n>]+"), " ") ?: "no response"),
                )

                if (index != commands.lastIndex && txDelayMs > 0) {
                    delay(txDelayMs)
                }
            }
        }
    }
}
```

Holding `deviceMutexes[device.id]` for the full sequence prevents normal sensor polling
from inserting `ATSH7DF` or another OBD request between the BCM header, diagnostic
session, action and cleanup commands.

## BleRuntime

In `BleRuntime.onEvent()`, rename the resolved value from `hex` to `command` and route
OBD controls through the sequence API:

```kotlin
val command = when (cmd.action) {
    "turn_on" -> c.command["on"]
    "turn_off" -> c.command["off"]
    "set_value" -> c.command["template"]?.let {
        formatCommand(it, prim?.doubleOrNull ?: 0.0)
    }
    "select_option" -> prim?.contentOrNull?.let { c.command[it] }
    "press" -> c.command["press"]
    else -> null
} ?: return

scope.launch {
    when (d.source) {
        Source.gatt_notify -> gatt.write(d, command)
        Source.obd -> obd.executeCommands(
            d,
            c.preCommands + command + c.postCommands,
        )
        else -> {}
    }
}
```

## Intended Nissan control config after patch

```yaml
controls:
  - key: sentra_interior_lights
    type: switch
    name: "Interior Lights"
    pre_commands:
      - "ATSH745"
      - "10C0"
    post_commands:
      - "1081"
      - "ATSH7DF"
    command:
      on: "30172001"
      off: "30172000"
```

The raw PID `17` mapping above is a same-era Nissan probe and still needs B16 validation.
The patch only fixes transport/sequencing; it does not assert that a particular Nissan
service-30 PID applies to the 2010 Sentra.

## Tests to add

- OBD control command is sent as ASCII plus CR, not hex-decoded binary.
- `pre_commands + action + post_commands` execute in exact order.
- Normal polling cannot interleave while the sequence mutex is held.
- GATT controls retain their existing raw-hex behavior.
- Missing/disconnected OBD connection returns cleanly.
- Each control response is logged so Nissan negative responses can be diagnosed.

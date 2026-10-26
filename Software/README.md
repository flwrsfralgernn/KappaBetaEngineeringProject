# iOS App Pseudocode

```text
WHEN APP OPENS
    Load saved sessions
    Show connection status and session list

WHEN USER CONNECTS
    Find wearable using Bluetooth
    Connect and read device status
    Request list of stored recordings

WHEN USER DOWNLOADS A RECORDING
    Receive and verify data chunks
    Save download progress
    Resume if the connection drops
    Save the completed recording
    Confirm successful receipt to the wearable

WHEN USER OPENS A SESSION
    Show ECG, heart rate, depth, and motion graphs
    Highlight warnings and deployment events

WHEN USER EXPORTS
    Create a shareable file
    Open the iPhone share sheet
```
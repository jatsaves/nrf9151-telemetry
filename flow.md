sequenceDiagram
    autonumber
    participant Main as main()
    participant MB as Modbus
    participant LTE as LTE / Modem
    participant Log as Error / Meas Log
    participant MQTT as MQTT Broker

    Note over Main: Device wakes up hourly

    alt Not First Boot
        Main->>MB: getMeasurement(&m)
        alt Modbus Fails
            MB-->>Main: Error (-5)
            Main->>Log: error_log_add() [Prints & Stores]
        end
    end

    Main->>LTE: lte_lc_normal() & lte_lc_connect()
    alt LTE Connection Fails (-116)
        LTE-->>Main: Error
        Main->>Log: error_log_add() [Logs reg, RSRP, time]
        Main->>LTE: lte_lc_power_off()
        Main->>Main: sleep_until_next_wakeup()
    else LTE Success
        Main->>LTE: date_time_update_async() (if needed)
        
        alt First Boot
            Main->>Log: error_log_add(EVENT_COLD_BOOT)
        else Modbus Success
            Main->>Log: measurement_log_add(&m)
        end
        
        Main->>MQTT: publish_logs()
        Note over MQTT: build_payload() -> mqtt_publish_method()
        
        alt Payload Build Fails
            MQTT-->>Main: -ENOMEM
            Main->>Log: Nuke & Pave (Wipe logs, add build error)
        else MQTT Publish Fails
            MQTT-->>Main: Error code
            Main->>Log: error_log_add() [Detailed step error]
        end

        Main->>LTE: lte_lc_power_off()
        Main->>Main: sleep_until_next_wakeup()
    end
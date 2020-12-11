// First object will be written in fields,
// second one in tags

var arr = [];
for (i = 0; i < msg.metadata.gateways.length; i++)
{
    arr.push(msg.metadata.gateways[i].rssi)
}
max_rssi = Math.max(...arr);
if(msg.port == 1)
{
    //Message structure for port 1 messages
    msg.payload = [{
    alt:            msg.payload.alt,
    epe:            msg.payload.epe,
    hdop:           msg.payload.hdop,
    lat:            msg.payload.lat,
    lon:            msg.payload.lon,
    lux:            msg.payload.lux,
    motion:         msg.payload.motion,
    port:           msg.port,
    satellites:     msg.payload.satellites,
    snr:            msg.payload.snr,
    time_to_fix:    msg.payload.time_to_fix,
    rssi:           max_rssi
    },
    { 
        devId: msg.dev_id,
        appId: msg.app_id
    }]
}
else if(msg.port == 12)
{
    //Message structure for port 1 messages
    msg.payload = [{
    battery:                msg.payload.battery,
    battery_low:            msg.payload.battery_low,
    port:                   msg.port,
    resetCause:             msg.payload.resetCause,
    accelerometer_error:    msg.payload.system_functions_errors.accelerometer_error,
    gps_fix_error:          msg.payload.system_functions_errors.gps_fix_error,
    gps_periodic_error:     msg.payload.system_functions_errors.gps_periodic_error,
    gps_triggered_error:    msg.payload.system_functions_errors.gps_triggered_error,
    gps_on_time_total:      msg.payload.gps_on_time_total,
    //humidity_error:         msg.payload.system_functions_errors.humidity_error,
    charging_status:            msg.payload.system_functions_errors.charging_status,
    //pressure_error:         msg.payload.system_functions_errors.charging_error,
    //temperature_error:      msg.payload.system_functions_errors.temperature_error,
    temperature:            msg.payload.temperature,
    rssi:                   max_rssi,
    accelx:                 msg.payload.accelx,
    accely:                 msg.payload.accely,
    accelz:                 msg.payload.accelz,
    pulse_counter:          msg.payload.pulse_counter,
    pulse_energy:           msg.payload.pulse_energy,
    pulse_voltage:          msg.payload.pulse_voltage,
    //lat:                    msg.payload.lat,
    //lon:                    msg.payload.lon,
    //time_to_fix:            msg.payload.time_to_fix,
    gps_resend:             msg.payload.gps_resend
    
    
    
    conductivity_ctz:     msg.payload.conductivity_ctz,
    salinity:             msg.payload.salinity,
    conductivity_no_comp: msg.payload.conductivity_no_comp,
                          
    oxygen_sat:           msg.payload.oxygen_sat,
    oxygen_mgL:           msg.payload.oxygen_mgL,
    oxygen_ppm:           msg.payload.oxygen_ppm,
                          
    bme_temp:             msg.payload.bme_temp,
    bme_pressure:         msg.payload.bme_pressure,
    bme_humid:            msg.payload.bme_humid,
                          
    significant_wh:       msg.payload.significant_wh,
    average_wh:           msg.payload.average_wh,
    average_period:       msg.payload.average_period,
    },
    { 
        devId: msg.dev_id,
        appId: msg.app_id
    }]
}
else if(msg.port == 2)
{
    //Message structure for port 2 messages
    msg.payload = [{
    battery:                msg.payload.battery,
    battery_low:            msg.payload.battery_low,
    port:                   msg.port,
    resetCause:             msg.payload.resetCause,
    accelerometer_error:    msg.payload.system_functions_errors.accelerometer_error,
    gps_fix_error:          msg.payload.system_functions_errors.gps_fix_error,
    gps_periodic_error:     msg.payload.system_functions_errors.gps_periodic_error,
    gps_triggered_error:    msg.payload.system_functions_errors.gps_triggered_error,
    humidity_error:         msg.payload.system_functions_errors.humidity_error,
    light_error:            msg.payload.system_functions_errors.light_error,
    pressure_error:         msg.payload.system_functions_errors.pressure_error,
    temperature_error:      msg.payload.system_functions_errors.temperature_error,
    temperature:            msg.payload.temperature,
    vbus:                   msg.payload.vbus,
    rssi:           max_rssi
    },
    { 
        devId: msg.dev_id,
        appId: msg.app_id
    }]
}
else if(msg.port == 3)
{
    msg.payload = [{
    port:                   msg.port
    },
    { 
        devId: msg.dev_id,
        appId: msg.app_id
    }]
}
else if(msg.port == 11)
{
    //For this port message we have to read 5 different locations, 
    //each one with its own timestamp, format it and sent it into Influxdb node. 
    //Influxdb enables us to write several points at once,
    //as seen on node-red documentation website 
    //  https://flows.nodered.org/node/node-red-contrib-influxdb) 

    // If msg.payload is an array of arrays, it will be written as a series of points containing fields and tags.

    //That means that we need 5 arrays, each will contain two objects. 
    // First object will contain value fields that we would like to write (lat and lon in this case), 
    // second object will contain dev id as usual.
    // We also need time value in first object because each lat and lon pair has different timestamps.
    // Injected epoch timestamp needs to be 19 digits in order to work with Grafana.

    // Below code creates functionality that is described above:

    

    //clear up payload, we will get our data from payload_fields
    msg.payload = []
    
    // Locations actualy contains a string not an js object so we need to parse it first.
    var parsed_locations = JSON.parse(msg.payload_fields.locations)
    
    //iterate through payload_fields, there should be only 5 packets, but we can make this universal
    for (i = 0; i < parsed_locations.length; i++)
    {
       
        // Make sure that time is correct length so that grafana accepts it
        var epoch_number = parsed_locations[i].time
        var epoch_length = epoch_number.toString().length
        var power_of_ten = 19 - epoch_length
        var correct_grafana_epoch = epoch_number * Math.pow(10, power_of_ten)

        
        //fill up packet that we will push into our msg.payload object
        var lat_lon_packet = [{
            port:   msg.port,
            lat:    parsed_locations[i].lat,
            lon:    parsed_locations[i].lon,
            time:   correct_grafana_epoch,
            rssi:   max_rssi
        },
        { 
            devId: msg.dev_id,
            appId: msg.app_id
        }]
        
        msg.payload.push(lat_lon_packet)
    }
}


return msg;

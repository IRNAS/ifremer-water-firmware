import json

data = {}

""" Below values can be changed to user preferences """
data['system_status_interval'] = 15
data['gps_periodic_interval']   = 30 # 30
data['gps_cold_fix_timeout']    = 200
data['gps_hot_fix_timeout']     = 60
data['gps_min_fix_time']        = 5
data['gps_min_ehpe']            = 50
data['gps_hot_fix_retry']       = 5
data['gps_cold_fix_retry']      = 5
data['gps_fail_retry']          = 1
data['magnetic_declination']    = 4.53              


""" Below values should not be changed """
system_functions = {}
system_functions['accelerometer_enabled']   = False  # Do not change
system_functions['light_enabled']           = False  # Do not change
system_functions['temperature_enabled']     = False  # Do not change
system_functions['humidity_enabled']        = False  # Do not change
system_functions['charging_enabled']        = True   # Do not change
data['system_functions'] = system_functions

lorawan_datarate_adr = {}
lorawan_datarate_adr["datarate"]= 3                  # Do not change
lorawan_datarate_adr["confirmed_uplink"]    = False  # Do not change
lorawan_datarate_adr["adr"]                 = False  # Do not change
data['lorawan_datarate_adr'] = lorawan_datarate_adr  # Do not change

gps_settings = {}
gps_settings['d3_fix']          = False             # Do not change
gps_settings['fail_backoff']    = False             # Do not change
gps_settings['hot_fix']         = True              # Do not change
gps_settings['fully_resolved']  = False             # Do not change
data['gps_settings'] = gps_settings

data['system_voltage_interval'] = 30                # Do not change
data['gps_charge_min']          = 0                 # Unused
data['system_charge_min']       = 0                 # Unused
data['system_charge_max']       = 5000              # Do not change
data['system_input_charge_min'] = 10000             # Unused
data['pulse_threshold']         = 0                 # Unused
data['pulse_on_timeout']        = 0                 # Unused
data['pulse_min_interval']      = 0                 # Unused
data['gps_accel_z_threshold']   = 0                 # Unused
data['fw_version']              = 0                 # Can not set
data['gps_triggered_interval']  = 0                 # Unused
data['gps_triggered_threshold'] = 15                # Unused
data['gps_triggered_duration']  = 15                # Unused

json_data = json.dumps(data)
print(json_data)


```
external_components: 
  - source:
      type: git
      url: https://github.com/hubecker/esp32p4_camera
      ref: main
    components: [tesp32p4_camera]
    refresh: 0s

tab5_camera:
  id: my_esp32p4_camera
  name: "esp32p4 Camera"
  i2c_id: bsp_bus
  external_clock:
    pin: GPIO36
    frequency: 20MHz


switch:
  - platform: gpio
    id: cam_rst
    name: "Camera Reset"
    pin:
      pi4ioe5v6408: pi4ioe1
      number: 6
    restore_mode: ALWAYS_ON
    on_turn_on:
      - delay: 100ms
      - logger.log: "Camera reset activated" 
```

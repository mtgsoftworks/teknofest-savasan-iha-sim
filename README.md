# TEKNOFEST Savasan IHA Simulasyon Altyapisi

Kurulu stack:
- WSL2 + Ubuntu-24.04
- PX4 SITL + Gazebo Harmonic
- ROS 2 Jazzy + MAVROS

## Ilk Kurulum (Yeni Makine)

Modelleri ve world dosyasini PX4'e kopyala:

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "chmod +x /mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/*.sh && /mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/install_models_to_px4.sh"
```

Bu komut ayni zamanda `4022_gz_rc_cessna_mono_cam` airframe dosyasini da PX4'e kopyalar.
Ek olarak `teknofest_competition.sdf` world plugin seti (AirPressure/AirSpeed/Magnetometer dahil) PX4 world klasorune senkronlanir.

## Sabit Kanat (Fixed-Wing) Pipeline

Tek komutla tam akis:
- PX4 + Gazebo + MAVROS (rc_cessna_mono_cam + airspeed sensor)
- Rosbridge
- Model spawn + IMU/NavSat/AirPressure/Airspeed/Camera bridge (deterministik baslangic)
- Geofence node (boundary + HSS ihlal kontrolu)
- Telemetry logger (CSV log)
- Offboard waypoint gorevi

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/run_fw_pipeline.sh"
```

Waypoint override:

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "WAYPOINTS='0,0,30;100,0,30;100,50,30;0,0,30' /mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/run_fw_pipeline.sh"
```

## Quadrotor Pipeline (x500)

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/run_savasan_iha_pipeline.sh"
```

## HSS Bolgesi Yonetimi

Spawn:
```bash
HSS_NAME=hss_zone_2 HSS_X=400 HSS_Y=50 scripts/spawn_hss_zone.sh
```

Kaldirma:
```bash
HSS_NAME=hss_zone_2 scripts/remove_hss_zone.sh
```

## Build ve Test

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "cd /mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select offboard_takeoff && colcon test --packages-select offboard_takeoff && colcon test-result --verbose"
```

## ROS 2 Node'lari

| Node | Komut | Aciklama |
|---|---|---|
| `fw_mission` | `ros2 run offboard_takeoff fw_mission` | Sabit kanat offboard waypoint gorevi |
| `geofence` | `ros2 run offboard_takeoff geofence` | Boundary + HSS ihlal kontrolu, RTL tetikleme |
| `telemetry_logger` | `ros2 run offboard_takeoff telemetry_logger` | CSV telemetry log |
| `offboard_mission` | `ros2 run offboard_takeoff offboard_mission` | Quadrotor waypoint gorevi |
| `offboard_takeoff` | `ros2 run offboard_takeoff offboard_takeoff` | Basit kalkis testi |

## Gazebo Modelleri

| Model | Aciklama |
|---|---|
| `rc_cessna_mono_cam` | Sabit kanat + asagi bakan kamera + airspeed sensor |
| `teknofest_runway` | 100m x 6m pist |
| `teknofest_qr_target` | 2m x 2m QR hedef tahtasi |
| `teknofest_hss_zone` | Kirmizi silindir yasak bolge (50m radius, 100m yukseklik) |
| `teknofest_boundary_marker` | Turuncu sinir isaretcisi |

## QR Ekibi Icin

Kamera topic, QR hedef konumlari ve test komutlari icin:
-> `docs/QR_EKIBI_ARAYUZ.md`

## Durdur

```powershell
wsl -d Ubuntu-24.04 -- bash -lc "/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/scripts/stop_px4_mavros.sh"
```

## Loglar

```bash
tail -n 80 /tmp/px4_fw_sitl.log
tail -n 80 /tmp/mavros_fw.log
```

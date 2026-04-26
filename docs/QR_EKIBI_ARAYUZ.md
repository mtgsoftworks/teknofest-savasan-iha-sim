# QR Ekibi Simülasyon Arayüzü Dokümanı

Bu doküman, QR detection ekibine simülasyon tarafından sağlanan kamera, model ve topic bilgilerini içerir.

## Kamera Modeli

- **Model adı:** `rc_cessna_mono_cam`
- **Kamera tipi:** Monocular (mono_cam)
- **Kamera konumu:** Burnun altına monte, **aşağı bakıyor** (pitch = 90°)
- **Kamera bağlantısı:** `camera_link` → `base_link` fixed joint

## Gazebo Kamera Topic'leri

Gazebo (gz) tarafındaki ham topic'ler:

```
Image:
/world/teknofest_competition/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/image

CameraInfo:
/world/teknofest_competition/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/camera_info
```

## ROS 2 Bridge Sonrası Topic'ler

`ros_gz_bridge` ile ROS 2'ye köprülendiğinde:

```
sensor_msgs/msg/Image       → aynı topic adı
sensor_msgs/msg/CameraInfo  → aynı topic adı
```

Bridge komutu (`mcp_reset_spawn_fw_sensor.sh` tarafından otomatik çalıştırılır):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/teknofest_competition/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/image"@sensor_msgs/msg/Image[gz.msgs.Image \
  "/world/teknofest_competition/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/camera_info"@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

## QR Hedef Modelleri

World dosyasında 2 adet QR hedefi tanımlıdır:

| Model Adı | Konum (x, y, z) | Boyut |
|---|---|---|
| `qr_target` | (200, 30, 0) | 2m × 2m |
| `qr_target_2` | (400, -30, 0) | 2m × 2m |

- QR board yerde yatay konumda, 0.01m yükseklikte
- Her hedefin 4 tarafında 45° açılı 3m yüksekliğinde plakalar var
- Board material'ı beyaz (QR texture eklenmesi gerekiyor)

## World Bilgileri

- **World adı:** `teknofest_competition`
- **World dosyası:** `worlds/teknofest_competition.sdf`
- **Uçuş alanı:** 600m × 400m (boundary marker'larla işaretli)
- **Koordinat sistemi:** Lokal NED (x=ileri, y=sol, z=yukarı Gazebo'da)

## Hızlı Test

QR ekibi kamera görüntüsünü şu komutla test edebilir:

```bash
# Stack çalışırken:
ros2 topic echo "/world/teknofest_competition/model/rc_cessna_mono_cam_0/link/camera_link/sensor/camera/image" --once

# Görüntü almak için rqt veya ros2 image_view:
ros2 run rqt_image_view rqt_image_view
```

## NOT

- QR texture henüz model'e eklenmemiş (beyaz board). QR ekibi kendi texture'ını `teknofest_qr_target/model.sdf` içine ekleyebilir.
- Kamera parametreleri (FOV, çözünürlük) `mono_cam` model'inin varsayılanlarına bağlıdır. Özelleştirme gerekirse `rc_cessna_mono_cam/model.sdf` güncellenmelidir.

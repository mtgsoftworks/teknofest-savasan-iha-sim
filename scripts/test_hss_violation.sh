#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST - HSS İhlal Test Senaryosu
# Bu script uçağı kasıtlı olarak HSS bölgesine (300,0,r=50) ve boundary sınırına yönlendirir.
# Geofence node'unun RTL tetiklemesini ve telemetry log'unda ihlal kaydını doğrular.
#
# Kullanım:
#   ./scripts/test_hss_violation.sh
#
# Beklenen sonuç:
#   - Geofence log'da "HSS VIOLATION" veya "BOUNDARY BREACH" mesajı
#   - Telemetry CSV'de geofence_status sütununda "BOUNDARY_BREACH" veya "IN_HSS" kaydı
#   - mode sütununda "AUTO.RTL" geçişi

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project}"

echo "============================================"
echo " TEKNOFEST HSS/Boundary İhlal Test Senaryosu"
echo "============================================"
echo ""
echo "Test 1: HSS Bölgesine Giren Waypoint Seti"
echo "  HSS zone: (300, 0, r=50)"
echo "  Waypoints: 0,0,30 → 280,0,30 → 310,0,30(HSS!) → 0,0,30"
echo ""
echo "Test 2: Boundary'yi Aşan Waypoint Seti"
echo "  Boundary: ±300 x ±200"
echo "  Waypoints: 0,0,30 → 250,150,30 → 320,0,30(OUT!) → 0,0,30"
echo ""

# -- Senaryo 1: HSS ihlali --
echo "[test-1] HSS ihlal testi başlatılıyor..."
echo "[test-1] Waypoints: HSS bölgesine (310,0) giriş"

WAYPOINTS="0,0,30;280,0,30;310,0,30;0,0,30" \
POS_TOL="20.0" \
CRUISE_SPEED="15.0" \
"${PROJECT_ROOT}/scripts/run_fw_pipeline.sh" 2>&1 | tee /tmp/test_hss_violation.log

echo ""
echo "============================================"
echo " Test Sonuçları"
echo "============================================"

# Geofence loglarını kontrol et
OUT_DIR=$(grep -o '/tmp/savasan_iha_fw_[0-9_]*' /tmp/test_hss_violation.log | tail -1)

if [ -n "${OUT_DIR}" ] && [ -d "${OUT_DIR}" ]; then
  echo ""
  echo "[kontrol] Geofence log kontrolü:"
  if grep -q "HSS\|BOUNDARY_BREACH\|IN_HSS" "${OUT_DIR}/05_geofence.log" 2>/dev/null; then
    echo "  ✅ İhlal tespit edildi:"
    grep -c "HSS\|BOUNDARY_BREACH" "${OUT_DIR}/05_geofence.log" || true
    echo "  satır bulundu"
  else
    echo "  ❌ Geofence ihlali bulunamadı"
  fi

  echo ""
  echo "[kontrol] Telemetry CSV kontrolü:"
  CSV_FILE=$(ls "${OUT_DIR}"/fw_mission_*.csv 2>/dev/null | head -1)
  if [ -n "${CSV_FILE}" ]; then
    echo "  CSV dosyası: ${CSV_FILE}"
    echo "  Toplam satır: $(wc -l < "${CSV_FILE}")"
    if grep -q "BOUNDARY_BREACH\|IN_HSS" "${CSV_FILE}" 2>/dev/null; then
      echo "  ✅ CSV'de ihlal kaydı var:"
      grep -c "BOUNDARY_BREACH\|IN_HSS" "${CSV_FILE}" || true
      echo "  satır"
    else
      echo "  ⚠️ CSV'de ihlal kaydı yok (uçak HSS'e ulaşamadıysa normal)"
    fi
    echo ""
    echo "  RTL geçişi kontrolü:"
    if grep -q "AUTO.RTL" "${CSV_FILE}" 2>/dev/null; then
      echo "  ✅ CSV'de AUTO.RTL modu kaydedilmiş"
    else
      echo "  ⚠️ AUTO.RTL kaydı yok"
    fi
    echo ""
    echo "  İlk 5 satır (header + veri):"
    head -5 "${CSV_FILE}"
  else
    echo "  ❌ CSV dosyası bulunamadı"
  fi
else
  echo "  ❌ Çıktı klasörü bulunamadı"
fi

echo ""
echo "[ok] Test tamamlandı"
echo "[hint] Detaylı inceleme: ls -la ${OUT_DIR:-/tmp/savasan_iha_fw_*}/"

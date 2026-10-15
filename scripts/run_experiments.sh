#!/usr/bin/env bash
# Deneyleri koşar: her senaryo, her koşu temiz başlayarak (yeni Gazebo, boş SLAM haritası) ve
# pencere açmadan (gui:=false rviz:=false) art arda tekrarlanır.
#
# Kullanım:
#   scripts/run_experiments.sh [-r tekrar] [-o klasör] [-t saniye] [senaryo ...] [-- argüman:=değer ...]
#
#   scripts/run_experiments.sh             3 senaryo × 5 tekrar, sonuçlar <depo>/results
#   scripts/run_experiments.sh -r 2 3      yalnızca senaryo 3, 2 tekrar
#   scripts/run_experiments.sh -o results/zor 4 -- goal_timeout:=120.0
#
#   -r  senaryo başına tekrar (varsayılan 5)
#   -o  sonuç klasörü (varsayılan <depo>/results)
#   -t  koşu başına gerçek zaman sınırı, s (varsayılan 900)
#   --  sonrası scenario.launch.py'ye olduğu gibi geçer
#
# Senaryo N'nin I. koşusu:
#   ros2 launch otonom_surus scenario.launch.py scenario:=N gui:=false rviz:=false \
#       results_dir:=<klasör> run_id:=rI
# Koşunun sonuçlarını mission_manager yazar (runs.csv, goals.csv, trajectories/sN_rI.csv,
# maps/sN_rI.*). Betik ayrıca her denemeyi experiments.csv'ye, launch çıktısını logs/sN_rI.log'a
# yazar. runs.csv'de satırı olan koşu atlanır: yarıda kalan deney aynı komutla kaldığı yerden
# sürer.
#
# Gerçek zaman sınırı neden: goal_timeout sim zamanıyla ölçülür; Gazebo çökerse /clock durur ve
# mission_manager sonsuza kadar bekler. Sınır aşılınca launch'a SIGINT gider, mission_manager
# sonuç yazmadan kapanır ve koşu experiments.csv'de recorded=0 ("kayıt yok") olarak kalır.
# Varsayılan 900 s, senaryo 2'nin en kötü durumunu (3 hedef × 180 s sim) da karşılar.
#
# Önce: source ~/ros2_ws/install/setup.bash. Aynı anda başka Gazebo ya da aynı ROS_DOMAIN_ID'de
# başka Nav2 çalışmamalı.

set -u

repo=$(cd "$(dirname "$0")/.." && pwd)
repeats=5
out=$repo/results
limit=900

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "$0"; exit 2; }
die() { echo "run_experiments: $*" >&2; exit 1; }

while getopts "r:o:t:h" opt; do
  case $opt in
    r) repeats=$OPTARG ;;
    o) out=$OPTARG ;;
    t) limit=$OPTARG ;;
    *) usage ;;
  esac
done
shift $((OPTIND - 1))
scenarios=()
while [ $# -gt 0 ] && [ "$1" != -- ]; do scenarios+=("$1"); shift; done
[ "${1:-}" = -- ] && shift
extra=("$@")
[ ${#scenarios[@]} -eq 0 ] && scenarios=(1 2 3)

[[ $repeats =~ ^[1-9][0-9]*$ ]] || die "-r pozitif tamsayı olmalı: $repeats"
[[ $limit =~ ^[1-9][0-9]*$ ]] || die "-t pozitif tamsayı (s) olmalı: $limit"
command -v ros2 > /dev/null || die "ros2 bulunamadı; önce: source /opt/ros/humble/setup.bash"
ros2 pkg prefix otonom_surus > /dev/null 2>&1 \
  || die "otonom_surus paketi bulunamadı; önce: source ~/ros2_ws/install/setup.bash"
# T2: eski bir gzserver yeni koşuda robotun doğmasını engeller.
pgrep gzserver > /dev/null \
  && die "çalışan bir gzserver var (başka simülasyon açık mı?); kapat: pkill -9 gzserver gzclient"

mkdir -p "$out/logs" && out=$(cd "$out" && pwd) || die "sonuç klasörü oluşturulamadı: $out"
csv=$out/experiments.csv
[ -f "$csv" ] || echo "timestamp,scenario,run_id,exit_code,wall_s,recorded,note" > "$csv"

recorded() {  # runs.csv'de senaryo $1, run_id $2 satırı var mı
  [ -f "$out/runs.csv" ] && awk -F, -v s="$1" -v id="$2" \
    'NR > 1 && $2 == s && $3 == id { found = 1 } END { exit !found }' "$out/runs.csv"
}

# Launch, timeout'un açtığı kendi süreç grubunda çalışır; Ctrl+C ona ulaşmaz, burada iletilir.
pid=
stop() {
  echo; echo "run_experiments: durduruluyor (koşu kaydedilmeyecek)..." >&2
  if [ -n "$pid" ]; then
    kill -INT -- -"$pid" 2> /dev/null
    wait "$pid"
    pkill -9 -g "$pid"
  fi
  exit 130
}
trap stop INT TERM

total=$((${#scenarios[@]} * repeats))
n=0 ok=0
echo "Deney: senaryolar ${scenarios[*]}, $repeats tekrar, sınır $limit s/koşu -> $out"
for s in "${scenarios[@]}"; do
  for i in $(seq 1 "$repeats"); do
    n=$((n + 1))
    run=r$i
    tag=s${s}_$run  # mission_manager'ın dosya adlarındaki gibi
    if recorded "$s" "$run"; then
      echo "[$n/$total] $tag: runs.csv'de var, atlandı"
      ok=$((ok + 1))
      continue
    fi
    printf '[%d/%d] %s: ' "$n" "$total" "$tag"
    start=$(date +%s)
    timeout -s INT -k 60 "$limit" ros2 launch otonom_surus scenario.launch.py \
      scenario:="$s" gui:=false rviz:=false results_dir:="$out" run_id:="$run" "${extra[@]}" \
      > "$out/logs/$tag.log" 2>&1 &
    pid=$!
    wait "$pid"
    rc=$?
    pkill -9 -g "$pid"  # launch'tan geride kalan süreç olmasın (T2)
    pid=
    wall=$(($(date +%s) - start))

    if recorded "$s" "$run"; then
      rec=1 note=
      ok=$((ok + 1))
      awk -F, -v s="$s" -v id="$run" -v w="$wall" 'NR > 1 && $2 == s && $3 == id {
        printf "%s, %s/%s hedef, %s s (sim); launch %s s\n", $4, $5, $6, $7, w }' "$out/runs.csv"
    else
      rec=0
      case $rc in
        124 | 137) note="gerçek zaman sınırı ($limit s) aşıldı; koşu kesildi" ;;
        *) note="launch çıkış kodu $rc; sonuç yazılmadı" ;;
      esac
      echo "KAYIT YOK: $note (log: $out/logs/$tag.log)"
    fi
    echo "$(date +%Y-%m-%dT%H:%M:%S),$s,$run,$rc,$wall,$rec,$note" >> "$csv"
    sleep 5  # Gazebo ve DDS portları boşalsın
  done
done
echo "Bitti: $ok/$total koşunun sonucu runs.csv'de."
echo "Analiz: python3 $repo/scripts/analyze_results.py $out"

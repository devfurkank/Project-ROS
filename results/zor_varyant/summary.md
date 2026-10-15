# Deney sonuçları

`scripts/analyze_results.py` ile 2026-10-15 15:08 tarihinde `results/zor_varyant` klasöründen üretildi: 10 koşu (S4: 5, S5: 5).

## Senaryo başına özet

| Senaryo | Koşu | Başarılı | Ulaşılan hedef | Süre (s, sim) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma | Hedefte kalan hata (m) | Gerçek zaman çarpanı |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S4 — Hareketli engeller, 2 kat hızlı | 5 | 5/5 | 5/5 | 28.2 ± 0.2 | 5.87 ± 0.05 | 0.963 ± 0.009 | 0.31 ± 0.02 (en az 0.29) | 0 | 0.20 ± 0.04 (en çok 0.26) | 0.99 |
| S5 — Hareketli engeller, 3 kat hızlı | 5 | 5/5 | 5/5 | 28.1 ± 0.4 | 5.87 ± 0.01 | 0.964 ± 0.002 | 0.44 ± 0.02 (en az 0.42) | 0 | 0.22 ± 0.01 (en çok 0.23) | 0.99 |

Ortalama ± standart sapma (örneklem, n − 1). Süre, yol ve verim yalnızca başarılı koşulardan; hedefte kalan hata başarılı hedeflerden (durulan nokta → hedef; Nav2 toleransı 0.25 m); en küçük engel mesafesi (LiDAR → en yakın yüzey, engel ya da duvar) ve kurtarma sayısı (toplam) sonucu kaydedilen bütün koşulardan. Verim = kuş uçuşu / gidilen yol. Gerçek zaman çarpanı = sim süresi / gerçek süre.

## Hedef başına

| Senaryo | Hedef | Konum (x, y, yaw) | Başarılı | Süre (s, sim) | Yol (m) | Kurtarma | Kalan hata (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S4 | 1 | (4, 4, 90°) | 5/5 | 28.2 ± 0.2 | 5.87 ± 0.05 | 0 | 0.20 ± 0.04 |
| S5 | 1 | (4, 4, 90°) | 5/5 | 28.1 ± 0.4 | 5.87 ± 0.01 | 0 | 0.22 ± 0.01 |

## Başarısız ya da kaydı olmayan koşular

Yok: 10 koşunun hepsi başarılı.

## Bütün koşular

| Senaryo | Koşu | Sonuç | Hedef | Süre (s, sim) | Gerçek süre (s) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S4 | r1 | SUCCEEDED | 1/1 | 28.2 | 28.4 | 5.92 | 0.956 | 0.29 | 0 |
| S4 | r2 | SUCCEEDED | 1/1 | 28.2 | 28.4 | 5.86 | 0.966 | 0.30 | 0 |
| S4 | r3 | SUCCEEDED | 1/1 | 27.9 | 28.1 | 5.79 | 0.977 | 0.31 | 0 |
| S4 | r4 | SUCCEEDED | 1/1 | 28.3 | 28.5 | 5.90 | 0.959 | 0.35 | 0 |
| S4 | r5 | SUCCEEDED | 1/1 | 28.5 | 28.8 | 5.90 | 0.958 | 0.30 | 0 |
| S5 | r1 | SUCCEEDED | 1/1 | 28.3 | 28.6 | 5.89 | 0.961 | 0.45 | 0 |
| S5 | r2 | SUCCEEDED | 1/1 | 27.6 | 27.8 | 5.86 | 0.965 | 0.45 | 0 |
| S5 | r3 | SUCCEEDED | 1/1 | 28.4 | 28.5 | 5.87 | 0.964 | 0.42 | 0 |
| S5 | r4 | SUCCEEDED | 1/1 | 27.8 | 28.0 | 5.87 | 0.964 | 0.45 | 0 |
| S5 | r5 | SUCCEEDED | 1/1 | 28.4 | 28.6 | 5.87 | 0.964 | 0.42 | 0 |

## Şekiller

- `figures/metrics.png`
- `figures/trajectory_s4.png`
- `figures/map_s4.png`
- `figures/trajectory_s5.png`
- `figures/map_s5.png`

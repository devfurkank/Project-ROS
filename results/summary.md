# Deney sonuçları

`scripts/analyze_results.py` ile 2026-10-15 15:08 tarihinde `results` klasöründen üretildi: 15 koşu (S1: 5, S2: 5, S3: 5).

## Senaryo başına özet

| Senaryo | Koşu | Başarılı | Ulaşılan hedef | Süre (s, sim) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma | Hedefte kalan hata (m) | Gerçek zaman çarpanı |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 — Dağınık statik engeller | 5 | 5/5 | 5/5 | 27.9 ± 0.2 | 5.76 ± 0.01 | 0.981 ± 0.002 | 0.36 ± 0.01 (en az 0.35) | 0 | 0.23 ± 0.00 (en çok 0.23) | 1.00 |
| S2 — Zikzak koridor, sıralı hedefler | 5 | 5/5 | 15/15 | 92.6 ± 0.2 | 17.49 ± 0.02 | 0.686 ± 0.001 | 0.33 ± 0.00 (en az 0.33) | 1 | 0.24 ± 0.01 (en çok 0.25) | 1.00 |
| S3 — Hareketli engeller | 5 | 5/5 | 5/5 | 28.2 ± 0.2 | 5.90 ± 0.03 | 0.959 ± 0.005 | 0.48 ± 0.01 (en az 0.47) | 0 | 0.24 ± 0.02 (en çok 0.28) | 0.99 |

Ortalama ± standart sapma (örneklem, n − 1). Süre, yol ve verim yalnızca başarılı koşulardan; hedefte kalan hata başarılı hedeflerden (durulan nokta → hedef; Nav2 toleransı 0.25 m); en küçük engel mesafesi (LiDAR → en yakın yüzey, engel ya da duvar) ve kurtarma sayısı (toplam) sonucu kaydedilen bütün koşulardan. Verim = kuş uçuşu / gidilen yol. Gerçek zaman çarpanı = sim süresi / gerçek süre.

## Hedef başına

| Senaryo | Hedef | Konum (x, y, yaw) | Başarılı | Süre (s, sim) | Yol (m) | Kurtarma | Kalan hata (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | 1 | (4, 4, 90°) | 5/5 | 27.9 ± 0.2 | 5.76 ± 0.01 | 0 | 0.23 ± 0.00 |
| S2 | 1 | (4, 0, 90°) | 5/5 | 22.1 ± 0.1 | 3.84 ± 0.01 | 0 | 0.24 ± 0.00 |
| S2 | 2 | (4, 4, 180°) | 5/5 | 52.0 ± 0.2 | 9.98 ± 0.02 | 0 | 0.24 ± 0.02 |
| S2 | 3 | (0, 4, 180°) | 5/5 | 18.5 ± 0.2 | 3.67 ± 0.01 | 1 | 0.24 ± 0.01 |
| S3 | 1 | (4, 4, 90°) | 5/5 | 28.2 ± 0.2 | 5.90 ± 0.03 | 0 | 0.24 ± 0.02 |

## Başarısız ya da kaydı olmayan koşular

Yok: 15 koşunun hepsi başarılı.

## Bütün koşular

| Senaryo | Koşu | Sonuç | Hedef | Süre (s, sim) | Gerçek süre (s) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1 | r1 | SUCCEEDED | 1/1 | 28.1 | 28.2 | 5.76 | 0.983 | 0.37 | 0 |
| S1 | r2 | SUCCEEDED | 1/1 | 27.7 | 27.8 | 5.76 | 0.983 | 0.35 | 0 |
| S1 | r3 | SUCCEEDED | 1/1 | 27.9 | 27.9 | 5.77 | 0.980 | 0.36 | 0 |
| S1 | r4 | SUCCEEDED | 1/1 | 27.8 | 28.0 | 5.78 | 0.979 | 0.35 | 0 |
| S1 | r5 | SUCCEEDED | 1/1 | 28.1 | 28.2 | 5.76 | 0.982 | 0.36 | 0 |
| S2 | r1 | SUCCEEDED | 3/3 | 92.4 | 92.8 | 17.48 | 0.686 | 0.33 | 0 |
| S2 | r2 | SUCCEEDED | 3/3 | 92.9 | 93.2 | 17.48 | 0.687 | 0.33 | 0 |
| S2 | r3 | SUCCEEDED | 3/3 | 92.3 | 92.5 | 17.45 | 0.688 | 0.33 | 0 |
| S2 | r4 | SUCCEEDED | 3/3 | 92.7 | 93.1 | 17.52 | 0.685 | 0.34 | 0 |
| S2 | r5 | SUCCEEDED | 3/3 | 92.7 | 93.1 | 17.50 | 0.686 | 0.34 | 1 |
| S3 | r1 | SUCCEEDED | 1/1 | 28.5 | 28.6 | 5.90 | 0.958 | 0.47 | 0 |
| S3 | r2 | SUCCEEDED | 1/1 | 28.2 | 28.3 | 5.90 | 0.959 | 0.50 | 0 |
| S3 | r3 | SUCCEEDED | 1/1 | 28.2 | 28.4 | 5.92 | 0.956 | 0.48 | 0 |
| S3 | r4 | SUCCEEDED | 1/1 | 28.3 | 28.5 | 5.92 | 0.956 | 0.48 | 0 |
| S3 | r5 | SUCCEEDED | 1/1 | 27.9 | 28.1 | 5.85 | 0.967 | 0.47 | 0 |

## Şekiller

- `figures/metrics.png`
- `figures/trajectory_s1.png`
- `figures/map_s1.png`
- `figures/trajectory_s2.png`
- `figures/map_s2.png`
- `figures/trajectory_s3.png`
- `figures/map_s3.png`

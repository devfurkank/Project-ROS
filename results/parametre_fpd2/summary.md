# Deney sonuçları

`scripts/analyze_results.py` ile 2026-10-15 15:08 tarihinde `results/parametre_fpd2` klasöründen üretildi: 5 koşu (S2: 5).

## Senaryo başına özet

| Senaryo | Koşu | Başarılı | Ulaşılan hedef | Süre (s, sim) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma | Hedefte kalan hata (m) | Gerçek zaman çarpanı |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S2 — Zikzak koridor, sıralı hedefler | 5 | 0/5 | 10/15 | — | — | — | 0.25 ± 0.09 (en az 0.12) | 75 | 0.25 ± 0.02 (en çok 0.29) | 1.00 |

Ortalama ± standart sapma (örneklem, n − 1). Süre, yol ve verim yalnızca başarılı koşulardan; hedefte kalan hata başarılı hedeflerden (durulan nokta → hedef; Nav2 toleransı 0.25 m); en küçük engel mesafesi (LiDAR → en yakın yüzey, engel ya da duvar) ve kurtarma sayısı (toplam) sonucu kaydedilen bütün koşulardan. Verim = kuş uçuşu / gidilen yol. Gerçek zaman çarpanı = sim süresi / gerçek süre.

## Hedef başına

| Senaryo | Hedef | Konum (x, y, yaw) | Başarılı | Süre (s, sim) | Yol (m) | Kurtarma | Kalan hata (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| S2 | 1 | (4, 0, 90°) | 5/5 | 22.1 ± 0.1 | 3.85 ± 0.01 | 0 | 0.23 ± 0.01 |
| S2 | 2 | (4, 4, 180°) | 0/5 | — | — | 75 | — |
| S2 | 3 | (0, 4, 180°) | 5/5 | 16.2 ± 0.3 | 2.46 ± 0.11 | 0 | 0.26 ± 0.02 |

## Başarısız ya da kaydı olmayan koşular

| Senaryo | Koşu | Sonuç | Ulaşılan hedef | Neden |
| --- | --- | --- | --- | --- |
| S2 | r1 | FAILED | 2/3 | hedef 2 (4.00, 4.00): FAILED, hedefe 3.29 m kala |
| S2 | r2 | FAILED | 2/3 | hedef 2 (4.00, 4.00): FAILED, hedefe 2.98 m kala |
| S2 | r3 | FAILED | 2/3 | hedef 2 (4.00, 4.00): FAILED, hedefe 2.92 m kala |
| S2 | r4 | TIMEOUT | 2/3 | hedef 2 (4.00, 4.00): TIMEOUT, hedefe 3.21 m kala |
| S2 | r5 | FAILED | 2/3 | hedef 2 (4.00, 4.00): FAILED, hedefe 3.36 m kala |

## Bütün koşular

| Senaryo | Koşu | Sonuç | Hedef | Süre (s, sim) | Gerçek süre (s) | Yol (m) | Verim | En küçük engel mesafesi (m) | Kurtarma |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S2 | r1 | FAILED | 2/3 | 217.1 | 217.9 | 12.28 | — | 0.28 | 15 |
| S2 | r2 | FAILED | 2/3 | 217.6 | 218.4 | 11.59 | — | 0.20 | 15 |
| S2 | r3 | FAILED | 2/3 | 216.6 | 217.4 | 11.58 | — | 0.12 | 15 |
| S2 | r4 | TIMEOUT | 2/3 | 218.8 | 219.6 | 11.06 | — | 0.33 | 15 |
| S2 | r5 | FAILED | 2/3 | 217.8 | 218.6 | 11.07 | — | 0.33 | 15 |

## Şekiller

- `figures/metrics.png`
- `figures/trajectory_s2.png`
- `figures/map_s2.png`

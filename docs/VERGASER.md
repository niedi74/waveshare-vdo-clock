# Vergaser-Kenndaten (VW T2b)

Referenz für Einstellarbeiten/Werkstatt. Stand 13.8., von Karsten durchgegeben —
noch nicht durch eine zweite Quelle (Vergaser-Datenblatt) gegengeprüft.

| Kenngröße | Wert |
|---|---|
| Typ | Solex PDSIT |
| Venturi (Lufttrichter) | 32 mm |
| Hauptdüse (HD) | 142 |
| LDK (Leerlauf-Luftdüse/-korrekturdüse) | 90 |
| LLD | 63 |

**Hinweis (Karsten):** Der Weber-Vergaser (falls später verbaut) bietet mehr
Einstellmöglichkeiten als der Solex — bei einem Wechsel diese Tabelle
entsprechend erweitern/anpassen.

Diese Werte sind im Gerät **editierbar** (reine Referenz, keine Funktion
hängt daran):
- **WebGUI** (System-Tab, Karte „Vergaser-Kenndaten"): alle 5 Felder
  (Typ/Venturi/HD/LDK/LLD).
- **Touchscreen** (Setup 2 → Zeile „VERGASER" → eigene Seite): nur
  HD/LDK/LLD per +/-1-Tasten, auch unterwegs ohne WLAN nutzbar. Typ/Venturi
  ändern sich selten und bleiben WebGUI-only.

Jede Änderung landet als Zeile im System-Log (`/log`, Präfix `VERGASER`).
Bei Wechsel der Hardware (z. B. auf Weber mit mehr Einstellmöglichkeiten)
diese Datei zusätzlich als Referenz aktualisieren.

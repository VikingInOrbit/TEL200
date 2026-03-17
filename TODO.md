# TODO

## Tidsfrister (oppdatert 16. mars 2026)
- [ ] **Tirsdag 17. mars:** Test på fysisk YuMi-robot i lab (hovedtest)
- [ ] **Fredag 20. mars:** Siste testmulighet i lab (buffer/fallback)
- [ ] **Mandag 23. mars:** Muntlig presentasjon + endelig innlevering på Canvas

## Part 1 (RobotStudio Basics)
- [x] Fullføre Part 1 ferdig (oppgitt som 100%)
- [x] Lage kort video (ca. 1 min) til muntlig presentasjon
- [ ] Verifisere at Part 1-videoen er klar for visning 23. mars

## Part 2 (YuMi Application)
### RobotStudio/teknisk
- [x] Pakke ut og bruke `TEL200-YuMi-Lab.rspag`
- [x] Sette opp DI-signaler i kode/løsning:
  - [x] `di_cube`
  - [x] `di_cylinder`
  - [x] `di_prism`
  - [x] `di_EmergencySituation` (aktiv lav)
  - [x] `di_home`
- [ ] Smart Component: E-stop knapp som går opp/ned i simulering
- [ ] (Anbefalt) Smart Component: 4-fargeknappene som går opp/ned i simulering
- [x] Definere WorkObjects/Targets/Paths for objekthåndtering
- [x] Knapplogikk for objektflytting (toggle + stack mode)
- [x] Teste at `di_EmergencySituation` stopper/fortsetter som forventet
- [x] Teste at `di_home` fungerer som forventet
- [ ] Kjør full simulering med Station Logic og verifiser alle knapper
- [ ] Test via Virtual FlexPendant: `PP to Main` + stegvis verifisering
- [ ] (Frist: 17. mars, backup 20. mars) Kjøre og filme tilsvarende script på fysisk YuMi

### Leveranse Part 2
- [ ] Lage video for Part 2 (1–2 min til muntlig / ev. 2–3 min for dokumentasjon)
- [ ] Skrive Part 2-del i rapport (metode, implementasjon, resultater)
- [ ] Knytte løsningen opp mot teori fra pensum

## Part 3 (YuMi Challenge)
### Løsning
- [x] Velge challenge-idé
- [x] Implementere challenge-logikk i RAPID
- [ ] Finpusse demo-flyt for tydelig og kreativ presentasjon
- [ ] Verifisere stabil kjøring i simulering (og real hvis tilgjengelig)

### Leveranse Part 3
- [ ] Lage video for Part 3 (maks 3 min, med lyd/tekstforklaring)
- [ ] Skrive Part 3-del i rapport (idé, designvalg, implementasjon, resultat)
- [ ] Beskrive kreativitet/originalitet og kobling til læringsutbytte
- [ ] Eksportere og levere `Pack&Go` (`.rspag`) for challenge-løsningen

## Rapport (felles for Part 2 + 3)
- [ ] Én rapport per gruppe med alle navn på forside
- [ ] Maks 10 sider totalt (inkl. vedlegg)
- [ ] Bruke riktig mal (NMBU Word/LaTeX)
- [ ] Følge IMRaD-struktur:
  - [ ] Abstract
  - [ ] Introduction
  - [ ] Method
  - [ ] Results
  - [ ] Discussion
  - [ ] Conclusions
- [ ] Beskrive nok detaljer til at andre kan reprodusere prosjektet
- [ ] Tydelig kobling mellom praksis (lab) og teori (pensum)
- [ ] Korrekturlesing (språk, struktur, figurer, referanser)

## Endelig innlevering (Canvas)
- [ ] Last opp PDF-rapport
- [ ] Last opp video Part 3 (evt. tilgjengelig lenke)
- [ ] Last opp nødvendige videoer for presentasjon/krav
- [ ] Last opp `Pack&Go`-fil (`.rspag`)
- [ ] Verifisere at alle filer kan åpnes etter opplasting
- [ ] Levere før fristen 23. mars

## Muntlig presentasjon (10 min)
- [ ] Forberede presentasjon av Part 1 + Part 2
- [ ] Velge klipp (maks 3 min total videovisning i presentasjonen)
- [ ] Fordele taledeler i gruppa
- [ ] Kjør en full prøvepresentasjon

## Siste kvalitetssjekk
- [ ] Alle DI-navn i kode matcher oppgaven (`di_cube`, `di_cylinder`, `di_prism`, `di_EmergencySituation`, `di_home`)
- [ ] E-stop oppførsel er sikker og demonstrerbar
- [ ] Knapper i sim ser fysisk riktige ut (opp/ned) i demo
- [ ] Robotbaner er kollisjonsfrie og repeterbare
- [ ] Alle leveranser finnes i en egen mappe for enkel opplasting

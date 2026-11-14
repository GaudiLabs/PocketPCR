# Getting a PocketPCR in India

Rough prices, November 2026, at about ₹95 per €. All of them are estimates: get a quote before you order.

## Which route?

| Route | You get | Rough cost | When it makes sense |
|---|---|---|---|
| GaudiShop, assembled | Finished device with case and cable, free shipping | €179 ≈ ₹17,000 + import duty | You want it working, no tools |
| GaudiShop, DIY kit | Kit + the picture guide in this folder | €99 ≈ ₹9,500 + import duty | **Cheapest for one device** |
| Make your own boards | 5 sets of assembled boards from JLCPCB | ≈ ₹8,000–11,000 for 5 incl. duty, + ~₹1,000 per device in loose parts | 3 or more devices |

## Making your own boards

The design files are already in this repo; nothing has to be redesigned.

### Main board: JLCPCB (China), parts soldered on

- Gerbers: `PCB_Layouts/PocketPCR_PCB/RocketPCR/Gerber/` (`RocketPCR1_1.zip` is the newest set).
- Placement: `RocketPCR-bottom-pos.csv`. All ~40 surface-mount parts are on the **bottom** side, so it is a single-side assembly. The top side only carries the screen.
- Parts list: `PocketPCR_BOM.ods`. Parts are common (SAMD21G18A, TLV1117-33, 0805 passives, USB-C socket).
- Ask them to also fit **J2** (2x5, 1.27 mm SWD header). You need it once, to load the first software.
- Delivery to India: about 8–15 days.

### Heater board: aluminium, JLCPCB

- Gerbers: `PCB_Layouts/PocketPCR_BottomHeater/Gerber/` (**not** the `WrongProductionFiles` folder).
- Aluminium base, 1.6 mm, ~2 W/mK, black. One 0402 10 kΩ NTC (NTCG103JX103DTDS or equivalent).
- ⚠ The heating track must stay **0.16 mm ± 20 %**: its width sets the heater's resistance. JLCPCB lists 0.10 mm as its minimum on aluminium, so this is within its capability. Don't let any fab "adjust" it.

### Import duty

PCBs pay about 28–30 % on arrival (10 % basic customs duty + 10 % surcharge on that + 18 % IGST). The courier collects it on delivery, sometimes with a handling fee.

### Made-in-India alternatives

- **Lion Circuits** (Bengaluru): bare 2-layer boards from ₹799 for 5, 5–6 days. Assembly with parts sourcing is quote-only. No aluminium boards listed, so the heater board still needs JLCPCB or an IndiaMART metal-core supplier.
- **PCB Power** (Gujarat): good for bare boards. Assembly only for 100+ boards.

## Parts no board maker supplies

| Part | Where | Rough ₹ |
|---|---|---|
| 0.96″ SPI OLED (SSD1306), 30-pin FPC type, for the on-board connector | Robu.in, Robocraze, AliExpress | 200–300 |
| EC11 rotary encoder with push switch, 20 mm shaft | Robu.in, electronics markets | 30–80 |
| 3010 fan, 5 V, 2 wires | Robu.in, SP Road / Lamington Rd / Lajpat Rai Mkt / Ritchie St | 60–150 |
| M3x16 (4), M3x6 (2), M3 nuts (2), M3 hex spacers 10 mm (2) and 20 mm (2) | Assorted kit online, or any hardware shop | 150–400 a kit |
| Rubber feet, knob cap, spring, M3x25 nylon thumb screw | Robu.in / Amazon.in / hardware shop | 100–200 |
| **Heat block**: 19.5 mm aluminium round (EN AW-6082) with 5 tube wells and an M3 centre thread | Local lathe/CNC shop or JLCCNC, using `Parts/PocketPCRHeater_V1.PDF` | quote |
| Raspberry Pi Pico, as a one-time SWD programmer for the first software load | Robu.in, Robocraze | 400–600 |

⚠ The OLED type is our reading of the footprint name (`OLED_096_1_Flip`) and connector (`SFV30R`). Check the pinout against the KiCad schematic before you buy.

## Sources

- [Zbotic: PCB assembly in India, JLCPCB vs local](https://zbotic.in/pcb-assembly-service-india-jlcpcb-vs-local-fab-cost-guide/) (an Indian competitor, so it may favour itself)
- [JLCPCB aluminium board capabilities](https://jlcpcb.com/help/article/jlc-aluminum-substrates-are-now-available)
- [Lion Circuits: Make in India PCB](https://www.lioncircuits.com/make-in-india-pcb)
- [WHYPCB: JLCPCB customs duty in India](https://www.whypcb.com/blog/jlcpcb-customs-duty-india) (sells a competing import service)
- [GaudiShop: PocketPCR](https://gaudishop.ch/index.php/product/pocketpcr/)

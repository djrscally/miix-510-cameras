Scope (\_SB.PCI0.CIO2)
{
    /* Define ports for CIO2 device:
     - endpoint of port1 is connected to CAM1
     */

    Name (_DSD, Package () {
        ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
        Package () {
            Package () { "port@1", "PRT0" },
        }
    })

    Name (PRT1, Package () {
        ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package () { "reg", 1 }, /* csi 1 */
        },
        ToUUID ("dbb8e3e6-5886-4ba6-8795-1319f52a966b"),
        Package () {
            Package () { "endpoint@0", "EP10" },
        }
    })

    Name (EP10, Package() {
        ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package () {
            Package () { "reg", 0 },
            Package () { "clock-lanes", 0 },
            Package () { "data-lanes", Package () { 1 } },
            Package () { "remote-endpoint",
                Package() { \_SB.PCI0.CAM1, "port@0", "endpoint@0" }
            },
        }
    })

}

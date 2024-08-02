from cereal import car
from openpilot.selfdrive.car.chrysler.values import CAR

Ecu = car.CarParams.Ecu

FINGERPRINTS = {
  CAR.PACIFICA_2017_HYBRID: [
    {168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 515: 7, 516: 7, 517: 7, 518: 7, 520: 8, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 4, 571: 3, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 701: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 746: 5, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 788:3, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 908: 8, 924: 3, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 956: 8, 958: 8, 959: 8, 969: 4, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1225: 8, 1235: 8, 1242: 8, 1246: 8, 1250: 8, 1284: 8, 1537: 8, 1538: 8, 1562: 8, 1568: 8, 1856: 8, 1858: 8, 1860: 8, 1865: 8, 1875: 8, 1882: 8, 1886: 8, 1890: 8, 1892: 8, 2016: 8, 2024: 8},
  ],
  CAR.PACIFICA_2018: [
    {55: 8, 257: 5, 258: 8, 264: 8, 268: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 516: 7, 517: 7, 520: 8, 524: 8, 526: 6, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 656: 4, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 746: 5, 752: 2, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 882: 8, 897: 8, 924: 8, 926: 3, 937: 8, 947: 8, 948: 8, 969: 4, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1098: 8, 1100: 8, 1537: 8, 1538: 8, 1562: 8},
    {55: 8, 257: 5, 258: 8, 264: 8, 268: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 516: 7, 517: 7, 520: 8, 524: 8, 526: 6, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 4, 571: 3, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 656: 4, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 746: 5, 752: 2, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 882: 8, 897: 8, 924: 3, 926: 3, 937: 8, 947: 8, 948: 8, 969: 4, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1098: 8, 1100: 8, 1537: 8, 1538: 8, 1562: 8},
  ],
  CAR.PACIFICA_2020: [
    {
      55: 8, 179: 8, 181: 8, 257: 5, 258: 8, 264: 8, 268: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 650: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 676: 8, 678: 8, 680: 8, 683: 8, 703: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 847: 1, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 882: 8, 897: 8, 906: 8, 924: 8, 926: 3, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1223: 7, 1225: 8, 1235: 8, 1242: 8, 1246: 8, 1250: 8, 1251: 8, 1252: 8, 1284: 8, 1568: 8, 1570: 8, 1856: 8, 1858: 8, 1860: 8, 1863: 8, 1865: 8, 1875: 8, 1882: 8, 1886: 8, 1890: 8, 1891: 8, 1892: 8, 2016: 8, 2017:8, 2024: 8, 2025: 8
    }
  ],
  CAR.PACIFICA_2018_HYBRID: [
    {68: 8, 168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 680: 8, 701: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 736: 8, 737: 8, 746: 5, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 969: 4, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8},
    # based on 9ae7821dc4e92455|2019-07-01--16-42-55
    {168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 515: 7, 516: 7, 517: 7, 518: 7, 520: 8, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 701: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 746: 5, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 969: 4, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1225: 8, 1235: 8, 1242: 8, 1246: 8, 1250: 8, 1251: 8, 1252: 8, 1258: 8, 1259: 8, 1260: 8, 1262: 8, 1284: 8, 1537: 8, 1538: 8, 1562: 8, 1568: 8, 1856: 8, 1858: 8, 1860: 8, 1865: 8, 1875: 8, 1882: 8, 1886: 8, 1890: 8, 1891: 8, 1892: 8, 1898: 8, 1899: 8, 1900: 8, 1902: 8, 2016: 8, 2018: 8, 2019: 8, 2020: 8, 2023: 8, 2024: 8, 2026: 8, 2027: 8, 2028: 8, 2031: 8},
  ],
  CAR.PACIFICA_2019_HYBRID: [
    {168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 502:8, 503:8, 512: 8, 514: 8, 515: 7, 516: 7, 517: 7, 518: 7, 520: 8, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 626: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 680: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 736: 8, 737: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1538: 8},
    # Based on 0607d2516fc2148f|2019-02-13--23-03-16
    {
      168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 502:8, 503:8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 626: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1537: 8
    },
    # Based on 3c7ce223e3571b54|2019-05-11--20-16-14
    {
      168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 502:8, 503:8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 626: 8, 632: 8, 639: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1562: 8, 1570: 8
    },
    # Based on "8190c7275a24557b|2020-02-24--09-57-23"
    {
      168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 502:8, 503:8, 512: 8, 514: 8, 515: 7, 516: 7, 517: 7, 518: 7, 520: 8, 524: 8, 526: 6, 528: 8, 532: 8, 542: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 626: 8, 632: 8, 639: 8, 640: 1, 650: 8, 653: 8, 654: 8, 655: 8, 656: 4, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 683: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 711: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 738: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 793: 8, 794: 8, 795: 8, 796: 8, 797: 8, 798: 8, 799: 8, 800: 8, 801: 8, 802: 8, 803: 8, 804: 8, 805: 8, 807: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 847: 1, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 886: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 929: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1225: 8, 1235: 8, 1242: 8, 1246: 8, 1250: 8, 1251: 8, 1252: 8, 1258: 8, 1259: 8, 1260: 8, 1262: 8, 1284: 8, 1568: 8, 1570: 8, 1856: 8, 1858: 8, 1860: 8, 1863: 8, 1865: 8, 1875: 8, 1882: 8, 1886: 8, 1890: 8, 1891: 8, 1892: 8, 1898: 8, 1899: 8, 1900: 8, 1902: 8, 2015: 8, 2016: 8, 2017: 8, 2018: 8, 2019: 8, 2020: 8, 2023: 8, 2024: 8, 2026: 8, 2027: 8, 2028: 8, 2031: 8
    },
    {
      168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 650: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 683: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 711: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 738: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 793: 8, 794: 8, 795: 8, 796: 8, 797: 8, 798: 8, 799: 8, 800: 8, 801: 8, 802: 8, 803: 8, 804: 8, 805: 8, 807: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 840: 8, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 886: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8
    },
    {
      168: 8, 257: 5, 258: 8, 264: 8, 268: 8, 270: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 291: 8, 292: 8, 294: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 368: 8, 376: 3, 384: 8, 388: 4, 448: 6, 450: 8, 456: 4, 464: 8, 469: 8, 480: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 528: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 650: 8, 653: 8, 654: 8, 655: 8, 658: 6, 660: 8, 669: 3, 671: 8, 672: 8, 678: 8, 680: 8, 683: 8, 701: 8, 703: 8, 704: 8, 705: 8, 706: 8, 709: 8, 710: 8, 711: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 738: 8, 746: 5, 752: 2, 754: 8, 760: 8, 764: 8, 766: 8, 770: 8, 773: 8, 779: 8, 782: 8, 784: 8, 792: 8, 793: 8, 794: 8, 795: 8, 796: 8, 797: 8, 798: 8, 799: 8, 800: 8, 801: 8, 802: 8, 803: 8, 804: 8, 805: 8, 807: 8, 808: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 832: 8, 838: 2, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 878: 8, 882: 8, 886: 8, 897: 8, 906: 8, 908: 8, 924: 8, 926: 3, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 958: 8, 959: 8, 962: 8, 969: 4, 973: 8, 974: 5, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1082: 8, 1083: 8, 1098: 8, 1100: 8
    },
  ],
  CAR.JEEP_GRAND_CHEROKEE: [{
    55: 8, 168: 8, 181: 8, 256: 4, 257: 5, 258: 8, 264: 8, 268: 8, 272: 6, 273: 6, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 352: 8, 362: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 4, 571: 3, 579: 8, 584: 8, 608: 8, 618: 8, 624: 8, 625: 8, 632: 8, 639: 8, 656: 4, 658: 6, 660: 8, 671: 8, 672: 8, 676: 8, 678: 8, 680: 8, 683: 8, 684: 8, 703: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 738: 8, 746: 5, 752: 2, 754: 8, 760: 8, 761: 8, 764: 8, 766: 8, 773: 8, 776: 8, 779: 8, 782: 8, 783: 8, 784: 8, 785: 8, 788: 3, 792: 8, 799: 8, 800: 8, 804: 8, 806: 2, 808: 8, 810: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 831: 6, 832: 8, 838: 2, 840: 8, 844: 5, 847: 1, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 874: 2, 882: 8, 897: 8, 906: 8, 924: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 956: 8, 968: 8, 969: 4, 970: 8, 973: 8, 974: 5, 975: 8, 976: 8, 977: 4, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1062: 8, 1098: 8, 1100: 8, 1543: 8, 1562: 8, 1576: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  },
    # Based on c88f65eeaee4003a|2022-08-04--15-37-16
  {
    257: 5, 258: 8, 264: 8, 268: 8, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 344: 8, 352: 8, 362: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 4, 564: 4, 571: 3, 584: 8, 608: 8, 624: 8, 625: 8, 632: 8, 639: 8, 658: 6, 660: 8, 671: 8, 672: 8, 678: 8, 680: 8, 684: 8, 703: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 746: 5, 752: 2, 760: 8, 761: 8, 764: 8, 766: 8, 773: 8, 776: 8, 779: 8, 783: 8, 784: 8, 792: 8, 799: 8, 800: 8, 804: 8, 806: 2, 810: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 831: 6, 832: 8, 838: 2, 844: 5, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 882: 8, 897: 8, 924: 3, 937: 8, 947: 8, 948: 8, 969: 4, 974: 5, 977: 4, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1062: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1223: 8, 1235: 8, 1242: 8, 1252: 8, 1792: 8, 1798: 8, 1799: 8, 1810: 8, 1813: 8, 1824: 8, 1825: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1860: 8, 1862: 8, 1863: 8, 1872: 8, 1875: 8, 1879: 8, 1882: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1953: 8, 1968: 8, 1988: 8, 2000: 8, 2001: 8, 2004: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  }],
  CAR.JEEP_GRAND_CHEROKEE_2019: [{
    # Jeep Grand Cherokee 2019, including most 2020 models
    55: 8, 168: 8, 179: 8, 181: 8, 256: 4, 257: 5, 258: 8, 264: 8, 268: 8, 272: 6, 273: 6, 274: 2, 280: 8, 284: 8, 288: 7, 290: 6, 292: 8, 300: 8, 308: 8, 320: 8, 324: 8, 331: 8, 332: 8, 341: 8, 344: 8, 352: 8, 362: 8, 368: 8, 376: 3, 384: 8, 388: 4, 416: 7, 448: 6, 456: 4, 464: 8, 500: 8, 501: 8, 512: 8, 514: 8, 520: 8, 530: 8, 532: 8, 544: 8, 557: 8, 559: 8, 560: 8, 564: 8, 571: 3, 579: 8, 584: 8, 608: 8, 618: 8, 624: 8, 625: 8, 632: 8, 639: 8, 640: 1, 656: 4, 658: 6, 660: 8, 671: 8, 672: 8, 676: 8, 678: 8, 680: 8, 683: 8, 684: 8, 703: 8, 705: 8, 706: 8, 709: 8, 710: 8, 719: 8, 720: 6, 729: 5, 736: 8, 737: 8, 738: 8, 746: 5, 752: 2, 754: 8, 760: 8, 761: 8, 764: 8, 766: 8, 773: 8, 776: 8, 779: 8, 782: 8, 783: 8, 784: 8, 785: 8, 792: 8, 799: 8, 800: 8, 804: 8, 806: 2, 808: 8, 810: 8, 816: 8, 817: 8, 820: 8, 825: 2, 826: 8, 831: 6, 832: 8, 838: 2, 840: 8, 844: 5, 847: 1, 848: 8, 853: 8, 856: 4, 860: 6, 863: 8, 874: 2, 882: 8, 897: 8, 906: 8, 924: 8, 937: 8, 938: 8, 939: 8, 940: 8, 941: 8, 942: 8, 943: 8, 947: 8, 948: 8, 960: 4, 968: 8, 969: 4, 970: 8, 973: 8, 974: 5, 976: 8, 977: 4, 979: 8, 980: 8, 981: 8, 982: 8, 983: 8, 984: 8, 992: 8, 993: 7, 995: 8, 996: 8, 1000: 8, 1001: 8, 1002: 8, 1003: 8, 1008: 8, 1009: 8, 1010: 8, 1011: 8, 1012: 8, 1013: 8, 1014: 8, 1015: 8, 1024: 8, 1025: 8, 1026: 8, 1031: 8, 1033: 8, 1050: 8, 1059: 8, 1062: 8, 1098: 8, 1100: 8, 1216: 8, 1218: 8, 1220: 8, 1223: 8, 1225: 8, 1227: 8, 1235: 8, 1242: 8, 1250: 8, 1251: 8, 1252: 8, 1254: 8, 1264: 8, 1284: 8, 1536: 8, 1537: 8, 1538: 8, 1543: 8, 1545: 8, 1562: 8, 1568: 8, 1570: 8, 1572: 8, 1593: 8, 1856: 8, 1858: 8, 1860: 8, 1863: 8, 1865: 8, 1867: 8, 1875: 8, 1882: 8, 1890: 8, 1891: 8, 1892: 8, 1894: 8, 1896: 8, 1904: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  }],
}

FW_VERSIONS = {
  CAR.CHRYSLER_PACIFICA_2017_HYBRID: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68239262AH',
      b'68239262AI',
      b'68239262AJ',
      b'68239263AH',
      b'68239263AJ',
    ],
    (Ecu.srs, 0x744, None): [
      b'68238840AH',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'68226356AI',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68288309AC',
      b'68288309AD',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'68277480AV ',
      b'68277480AX ',
      b'68277480AZ ',
    ],
    (Ecu.hybrid, 0x7e2, None): [
      b'05190175BF',
      b'05190175BH',
      b'05190226AK',
    ],
  },
  CAR.CHRYSLER_PACIFICA_2018: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68227902AF',
      b'68227902AG',
      b'68227902AH',
      b'68227905AG',
      b'68360252AC',
    ],
    (Ecu.srs, 0x744, None): [
      b'68211617AF',
      b'68211617AG',
      b'68358974AC',
      b'68405937AA',
    ],
    (Ecu.abs, 0x747, None): [
      b'68222747AG',
      b'68330876AA',
      b'68330876AB',
      b'68352227AA',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672758AA',
      b'68226356AF',
      b'68226356AH',
      b'68226356AI',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68288891AE',
      b'68378884AA',
      b'68525338AA',
      b'68525338AB',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'68267018AO ',
      b'68267020AJ ',
      b'68303534AG ',
      b'68303534AJ ',
      b'68340762AD ',
      b'68340764AD ',
      b'68352652AE ',
      b'68352654AE ',
      b'68366851AH ',
      b'68366853AE ',
      b'68372861AF ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'68277370AJ',
      b'68277370AM',
      b'68277372AD',
      b'68277372AE',
      b'68277372AN',
      b'68277374AA',
      b'68277374AB',
      b'68277374AD',
      b'68277374AN',
      b'68367471AC',
      b'68380571AB',
    ],
  },
  CAR.CHRYSLER_PACIFICA_2020: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68405327AC',
      b'68436233AB',
      b'68436233AC',
      b'68436234AB',
      b'68436250AE',
      b'68529067AA',
      b'68594993AB',
      b'68594994AB',
    ],
    (Ecu.srs, 0x744, None): [
      b'68405565AB',
      b'68405565AC',
      b'68444299AC',
      b'68480707AC',
      b'68480708AC',
      b'68526663AB',
    ],
    (Ecu.abs, 0x747, None): [
      b'68397394AA',
      b'68433480AB',
      b'68453575AF',
      b'68577676AA',
      b'68593395AA',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672758AA',
      b'04672758AB',
      b'68417813AF',
      b'68540436AA',
      b'68540436AC',
      b'68540436AD',
      b'68598670AB',
      b'68598670AC',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68416742AA',
      b'68460393AA',
      b'68460393AB',
      b'68494461AB',
      b'68494461AC',
      b'68524936AA',
      b'68524936AB',
      b'68525338AB',
      b'68594337AB',
      b'68594340AB',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'68413871AD ',
      b'68413871AE ',
      b'68413871AH ',
      b'68413871AI ',
      b'68413873AH ',
      b'68413873AI ',
      b'68443120AE ',
      b'68443123AC ',
      b'68443125AC ',
      b'68496647AI ',
      b'68496647AJ ',
      b'68496650AH ',
      b'68496650AI ',
      b'68496652AH ',
      b'68526752AD ',
      b'68526752AE ',
      b'68526754AE ',
      b'68536264AE ',
      b'68700304AB ',
      b'68700306AB ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'68414271AC',
      b'68414271AD',
      b'68414275AC',
      b'68414275AD',
      b'68443154AB',
      b'68443155AC',
      b'68443158AB',
      b'68501050AD',
      b'68501051AD',
      b'68501055AD',
      b'68527221AB',
      b'68527223AB',
      b'68586231AD',
      b'68586233AD',
    ],
  },
  CAR.CHRYSLER_PACIFICA_2018_HYBRID: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68358439AE',
      b'68358439AG',
    ],
    (Ecu.srs, 0x744, None): [
      b'68358990AC',
      b'68405939AA',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672758AA',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68288309AD',
      b'68525339AA',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'68366580AI ',
      b'68366580AK ',
      b'68366580AM ',
    ],
    (Ecu.hybrid, 0x7e2, None): [
      b'05190226AI',
      b'05190226AK',
      b'05190226AM',
    ],
  },
  CAR.CHRYSLER_PACIFICA_2019_HYBRID: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68405292AC',
      b'68434956AC',
      b'68434956AD',
      b'68434960AE',
      b'68434960AF',
      b'68529064AB',
      b'68594990AB',
    ],
    (Ecu.srs, 0x744, None): [
      b'68405567AB',
      b'68405567AC',
      b'68453076AD',
      b'68480710AC',
      b'68526665AB',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672758AB',
      b'68417813AF',
      b'68540436AA',
      b'68540436AB',
      b'68540436AC',
      b'68540436AD',
      b'68598670AB',
      b'68598670AC',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68416741AA',
      b'68460392AA',
      b'68525339AA',
      b'68525339AB',
      b'68594341AB',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'68416680AE ',
      b'68416680AF ',
      b'68416680AG ',
      b'68444228AD ',
      b'68444228AE ',
      b'68444228AF ',
      b'68499122AD ',
      b'68499122AE ',
      b'68499122AF ',
      b'68526772AD ',
      b'68526772AH ',
      b'68599493AC ',
    ],
    (Ecu.hybrid, 0x7e2, None): [
      b'05185116AF',
      b'05185116AJ',
      b'05185116AK',
      b'05190240AP',
      b'05190240AQ',
      b'05190240AR',
      b'05190265AG',
      b'05190265AH',
      b'05190289AE',
      b'68540977AH',
      b'68540977AK',
      b'68597647AE',
    ],
  },
  CAR.JEEP_GRAND_CHEROKEE: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68243549AG',
      b'68302211AC',
      b'68302212AD',
      b'68302223AC',
      b'68302246AC',
      b'68331511AC',
      b'68331574AC',
      b'68331687AC',
      b'68331690AC',
      b'68340272AD',
    ],
    (Ecu.srs, 0x744, None): [
      b'68309533AA',
      b'68316742AB',
      b'68355363AB',
    ],
    (Ecu.abs, 0x747, None): [
      b'68252642AG',
      b'68306178AD',
      b'68336276AB',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672627AB',
      b'68251506AF',
      b'68332015AB',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68276201AG',
      b'68321644AB',
      b'68321644AC',
      b'68321646AC',
      b'68321648AC',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'05035920AE ',
      b'68252272AG ',
      b'68284455AI ',
      b'68284456AI ',
      b'68284477AF ',
      b'68325564AH ',
      b'68325565AH ',
      b'68325565AI ',
      b'68325618AD ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'05035517AH',
      b'68253222AF',
      b'68311218AC',
      b'68311223AF',
      b'68311223AG',
      b'68361911AE',
      b'68361911AF',
      b'68361911AH',
      b'68361916AD',
    ],
  },
  CAR.JEEP_GRAND_CHEROKEE_2019: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68402703AB',
      b'68402704AB',
      b'68402708AB',
      b'68402971AD',
      b'68454144AD',
      b'68454145AB',
      b'68454152AB',
      b'68454156AB',
      b'68516650AB',
      b'68516651AB',
      b'68516669AB',
      b'68516671AB',
      b'68516683AB',
    ],
    (Ecu.srs, 0x744, None): [
      b'68355363AB',
      b'68355364AB',
    ],
    (Ecu.abs, 0x747, None): [
      b'68408639AC',
      b'68408639AD',
      b'68499978AB',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672788AA',
      b'68456722AC',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68417279AA',
      b'68417280AA',
      b'68417281AA',
      b'68453431AA',
      b'68453433AA',
      b'68453435AA',
      b'68499171AA',
      b'68499171AB',
      b'68501183AA',
      b'68501186AA',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'05035674AB ',
      b'68412635AG ',
      b'68412660AD ',
      b'68422860AB',
      b'68449435AE ',
      b'68496223AA ',
      b'68504959AD ',
      b'68504959AE ',
      b'68504960AD ',
      b'68504993AC ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'05035707AA',
      b'68419672AC',
      b'68419678AB',
      b'68423905AB',
      b'68449258AC',
      b'68495807AA',
      b'68495807AB',
      b'68503641AC',
      b'68503644AC',
      b'68503664AC',
    ],
  },
  CAR.RAM_1500_5TH_GEN: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68294051AG',
      b'68294051AI',
      b'68294052AG',
      b'68294052AH',
      b'68294063AG',
      b'68294063AH',
      b'68294063AI',
      b'68434846AC',
      b'68434847AC',
      b'68434849AC',
      b'68434856AC',
      b'68434858AC',
      b'68434859AC',
      b'68434860AC',
      b'68453483AC',
      b'68453483AD',
      b'68453487AD',
      b'68453491AC',
      b'68453491AD',
      b'68453499AD',
      b'68453503AC',
      b'68453503AD',
      b'68453505AC',
      b'68453505AD',
      b'68453511AC',
      b'68453513AC',
      b'68453513AD',
      b'68453514AD',
      b'68505633AB',
      b'68510277AG',
      b'68510277AH',
      b'68510280AG',
      b'68510282AG',
      b'68510282AH',
      b'68510283AG',
      b'68527346AE',
      b'68527361AD',
      b'68527375AD',
      b'68527381AE',
      b'68527382AE',
      b'68527383AD',
      b'68527387AE',
      b'68527403AC',
      b'68527403AD',
      b'68546047AF',
      b'68631938AA',
      b'68631939AA',
      b'68631940AA',
      b'68631940AB',
      b'68631942AA',
      b'68631943AB',
    ],
    (Ecu.srs, 0x744, None): [
      b'68428609AB',
      b'68441329AB',
      b'68473844AB',
      b'68490898AA',
      b'68500728AA',
      b'68615033AA',
      b'68615034AA',
    ],
    (Ecu.abs, 0x747, None): [
      b'68292406AG',
      b'68292406AH',
      b'68432418AB',
      b'68432418AC',
      b'68432418AD',
      b'68436004AD',
      b'68436004AE',
      b'68438454AC',
      b'68438454AD',
      b'68438456AE',
      b'68438456AF',
      b'68535469AB',
      b'68535470AC',
      b'68548900AB',
      b'68586307AB',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672892AB',
      b'04672932AB',
      b'04672932AC',
      b'22DTRHD_AA',
      b'68320950AH',
      b'68320950AI',
      b'68320950AJ',
      b'68320950AL',
      b'68320950AM',
      b'68454268AB',
      b'68475160AE',
      b'68475160AF',
      b'68475160AG',
    ],
    (Ecu.eps, 0x75a, None): [
      b'21590101AA',
      b'21590101AB',
      b'68273275AF',
      b'68273275AG',
      b'68273275AH',
      b'68312176AE',
      b'68312176AG',
      b'68440789AC',
      b'68466110AA',
      b'68466110AB',
      b'68466113AA',
      b'68469901AA',
      b'68469907AA',
      b'68522583AA',
      b'68522583AB',
      b'68522584AA',
      b'68522585AB',
      b'68552788AA',
      b'68552789AA',
      b'68552790AA',
      b'68552791AB',
      b'68552794AA',
      b'68552794AD',
      b'68585106AB',
      b'68585107AB',
      b'68585108AB',
      b'68585109AB',
      b'68585112AB',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'05035699AG ',
      b'05035841AC ',
      b'05035841AD ',
      b'05036026AB ',
      b'05036065AE ',
      b'05036066AE ',
      b'05036193AA ',
      b'05149368AA ',
      b'05149591AD ',
      b'05149591AE ',
      b'05149592AE ',
      b'05149599AE ',
      b'05149600AD ',
      b'05149605AE ',
      b'05149846AA ',
      b'05149848AA ',
      b'05149848AC ',
      b'05190341AD',
      b'68378695AJ ',
      b'68378696AJ ',
      b'68378701AI ',
      b'68378702AI ',
      b'68378710AL ',
      b'68378742AI ',
      b'68378742AK ',
      b'68378748AL ',
      b'68378758AM ',
      b'68448163AJ',
      b'68448163AK',
      b'68448163AL',
      b'68448165AG',
      b'68448165AK',
      b'68455111AC ',
      b'68455119AC ',
      b'68455145AC ',
      b'68455145AE ',
      b'68455146AC ',
      b'68467915AC ',
      b'68467916AC ',
      b'68467936AC ',
      b'68500630AD',
      b'68500630AE',
      b'68500631AE',
      b'68502719AC ',
      b'68502722AC ',
      b'68502733AC ',
      b'68502734AF ',
      b'68502740AF ',
      b'68502741AF ',
      b'68502742AC ',
      b'68502742AF ',
      b'68539650AD',
      b'68539650AF',
      b'68539651AD',
      b'68586101AA ',
      b'68586105AB ',
      b'68629919AC ',
      b'68629922AC ',
      b'68629925AC ',
      b'68629926AC ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'05035706AD',
      b'05035842AB',
      b'05036069AA',
      b'05036181AA',
      b'05149536AC',
      b'05149537AC',
      b'05149543AC',
      b'68360078AL',
      b'68360080AL',
      b'68360080AM',
      b'68360081AM',
      b'68360085AJ',
      b'68360085AL',
      b'68360086AH',
      b'68360086AK',
      b'68384328AD',
      b'68384332AD',
      b'68445531AC',
      b'68445533AB',
      b'68445536AB',
      b'68445537AB',
      b'68466081AB',
      b'68466087AB',
      b'68484466AC',
      b'68484467AC',
      b'68484471AC',
      b'68502994AD',
      b'68502996AD',
      b'68520867AE',
      b'68520867AF',
      b'68520870AC',
      b'68540431AB',
      b'68540433AB',
      b'68551676AA',
      b'68629935AB',
      b'68629936AC',
    ],
  },
  CAR.RAM_HD_5TH_GEN: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68361606AH',
      b'68437735AC',
      b'68492693AD',
      b'68525485AB',
      b'68525487AB',
      b'68525498AB',
      b'68528791AF',
      b'68628474AB',
    ],
    (Ecu.srs, 0x744, None): [
      b'68399794AC',
      b'68428503AA',
      b'68428505AA',
      b'68428507AA',
    ],
    (Ecu.abs, 0x747, None): [
      b'68334977AH',
      b'68455481AC',
      b'68504022AA',
      b'68504022AB',
      b'68504022AC',
      b'68530686AB',
      b'68530686AC',
      b'68544596AC',
      b'68641704AA',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'04672895AB',
      b'04672934AB',
      b'56029827AG',
      b'56029827AH',
      b'68462657AE',
      b'68484694AD',
      b'68484694AE',
      b'68615489AB',
    ],
    (Ecu.eps, 0x761, None): [
      b'68421036AC',
      b'68507906AB',
      b'68534023AC',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'52370131AF',
      b'52370231AF',
      b'52370231AG',
      b'52370491AA',
      b'52370931CT',
      b'52401032AE',
      b'52421132AF',
      b'52421332AF',
      b'68527616AD ',
      b'M2370131MB',
      b'M2421132MB',
    ],
  },
  CAR.DODGE_DURANGO: {
    (Ecu.combinationMeter, 0x742, None): [
      b'68454261AD',
      b'68471535AE',
    ],
    (Ecu.srs, 0x744, None): [
      b'68355362AB',
      b'68492238AD',
    ],
    (Ecu.abs, 0x747, None): [
      b'68408639AD',
      b'68499978AB',
    ],
    (Ecu.fwdRadar, 0x753, None): [
      b'68440581AE',
      b'68456722AC',
    ],
    (Ecu.eps, 0x75a, None): [
      b'68453435AA',
      b'68498477AA',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'05035786AE ',
      b'68449476AE ',
    ],
    (Ecu.transmission, 0x7e1, None): [
      b'05035826AC',
      b'68449265AC',
    ],
  },
}

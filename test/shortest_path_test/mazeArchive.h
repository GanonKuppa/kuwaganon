#pragma once

#include <stdint.h>

namespace maze_archive {

// 北信越大会 2019
    class Hokushinetsu2019_HF {
      public:
        const uint32_t walls_vertical[31] = {0x00006ff9,0x00000bea,0x0000307f,0x00004fe0,0x0000affe,0x0000df08,0x0000000e,0x0000c618,0x00006f32,0x00001901,0x00000510,0x0000e122,0x00007082,0x0000ac52,0x00007ea2,0x0000ffff,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
        const uint32_t walls_horizontal[31] = {0x00004548,0x00003faa,0x0000ff08,0x00007e6a,0x000022c0,0x0000fde4,0x00001ff0,0x000065c8,0x00007a88,0x00003408,0x000028c8,0x00001d8e,0x00000bea,0x00000494,0x0000064a,0x0000ffff,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000,0x00000000};
        const uint8_t goal_x = 6;
        const uint8_t goal_y = 9;
    };


// 第30回全日本大会 2009 ハーフエキスパート決勝
    class AllJapan2009Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x07777777,0xfdefdef7,0x07ffdfec,0x06f7ade9,0x056fd34d,0x039daedc,0x07f7dfbc,0x05fdef74,0x03bf10b0,0x01000031,0x002100a9,0x055e5f64,0x0afadfd2,0x02b1cba8,0x0321df52,0x02b66be4,0x01ae1749,0x08af0422,0x05bd0106,0x027b768e,0x05f5bf46,0x00681efc,0x00680112,0x006f8028,0x046f9fb6,0x03edb9dd,0x05d09098,0x02917bec,0x0128bfd4,0x01749f91,0x06f9bfce};
        const uint32_t walls_horizontal[31] = {0x3d95fbec,0x6f536f58,0x61a6f712,0x20f59a18,0x50976830,0xcefe9a40,0xa1ddd680,0x818e8920,0x8c4e4048,0x040a0010,0x8c0d0020,0x800e8150,0xa1c16e28,0xcfe07f50,0x54ffcf00,0x290b5600,0x7ceb4800,0xac570040,0x5c6f2080,0x295ed090,0x157a8820,0x183ac640,0x3136ac20,0x51d26c10,0x2bf6aca8,0x75efaf40,0x79f3ef28,0xffffffff,0x00000000,0x00000000,0x00000000};
        const uint8_t goal_x = 24;
        const uint8_t goal_y = 15;
    };



// 第31回全日本大会 2010 ハーフエキスパート決勝
    class AllJapan2010Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x00fbad39,0x043f65bd,0x0758d016,0x03f94e1e,0x01b20668,0x08723980,0x01b4f77c,0x09b7faa8,0x05b60c50,0x05a602a1,0x05b6094e,0x01860276,0x05be05ae,0x031feb32,0x07cd45d6,0x07e4a06a,0x07ff1b6a,0x05fcb1d8,0x06575ebc,0x05927f5c,0x0575005e,0x05ea01ad,0x04f4026d,0x04c80161,0x04180069,0x04ff7f44,0x04a05f2e,0x04a2a740,0x04a0ff78,0x04a10f58,0x04b17f78};
        const uint32_t walls_horizontal[31] = {0x535d52aa,0x72c6057a,0x12c0a2a0,0x0eca2b40,0x4d800650,0x126e453d,0xec81fa4a,0x02d91614,0x0320a93c,0x01ffd446,0x13f0ac24,0x48017226,0x3800ff79,0x40050011,0x7ffa4032,0x97d40074,0xa9e7ff58,0x77508059,0x7cb10950,0xb8409200,0xaa84c8a4,0xa9086880,0x7c002204,0x03080825,0xffe40922,0x7ef27eda,0x000850b6,0xffffffff,0x00000000,0x00000000,0x00000000};
        const uint8_t goal_x = 25;
        const uint8_t goal_y = 22;
    };


// 第32回全日本大会 2011 ハーフエキスパート決勝
    class AllJapan2011Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x61adb03b,0x1194497c,0x2ff03780,0x2001c000,0x0008a002,0x20045000,0x4000ff80,0xb0771578,0x203a2a94,0x001854aa,0x00202d54,0x3f915528,0x4a3fad50,0x04857ea0,0x160a8148,0x04d500ac,0x029b814e,0x26d40004,0x48280104,0xa31001f0,0x12a7fe20,0x0d4aa642,0x8ac54e4b,0x154a1766,0xbafffffa,0x03fff7f4,0x02568000,0x387ea000,0x5b902002,0x293cc7f4,0x569571aa};
        const uint32_t walls_horizontal[31] = {0x542abbfc,0x2a18b5aa,0x55502a54,0x3ca654a8,0x00e6a854,0x000447a8,0x0024bad6,0x00e4092a,0x001fc252,0x8000022e,0x7c000552,0x54600aa8,0x6d601696,0x09402900,0x1481d704,0x230007a4,0x38000c56,0x201e2e54,0x10141988,0x016024fa,0x00c4c000,0x640bee02,0x301c5f00,0x0da8a000,0x1145a002,0x78c80002,0x4d362002,0x20154ffa,0x50457464,0x2ed16694,0x5525ad2e};
        const uint8_t goal_x = 25;
        const uint8_t goal_y = 28;
    };

// 第33回全日本大会 2012 ハーフエキスパート決勝
    class AllJapan2012Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x3bfd9fff,0x5dfb3ffc,0x2bf19ff8,0x54020ff0,0x280007e0,0x540003c0,0x2800c180,0x540180c0,0x2800c1e0,0x57f083f0,0x2df947f8,0x54788ffc,0x38745ffe,0x102a9000,0x00551000,0x202a1000,0x70145ffc,0x3e28dff8,0x4c555ffc,0x462a50e0,0x0fd641c0,0x00aec01c,0x21ffff80,0x0eee0000,0x1f44000c,0x4da80000,0x1c5c0000,0x003eff8c,0x0e1c73c0,0x000ee780,0x3dddffda};
        const uint32_t walls_horizontal[31] = {0x5fffdffc,0x36f7effc,0x369007f8,0x369803f0,0x3ab001e0,0x7fe000c0,0x0fc00000,0x00200000,0x00580180,0x002803c0,0x305007e0,0x6029eff0,0x00501ff8,0x0001dffe,0x0002a03e,0x3fb55070,0x7f0abff0,0x04155fee,0x022aa7fc,0x011543f8,0x632aa000,0x74114000,0x3a0fe000,0x3d1ff000,0x78aff800,0x5001fffe,0x2803e000,0x50f9c000,0x3ffc4000,0x7968e000,0xffffffff};
        const uint8_t goal_x = 22;
        const uint8_t goal_y = 26;
    };


// 第34回全日本大会 2013 ハーフエキスパート決勝
    class AllJapan2013Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x7d0ff575,0x2a8fe200,0x101fc480,0x083fe000,0x67dfe000,0x798006fe,0x64c00408,0x08000210,0x100007e0,0x241fe000,0x2f9ff800,0x100fc3f0,0x0c07e9cc,0x64c3f880,0xf9e03100,0x67f04300,0x0bf8f798,0x17fe7ffc,0x20003ff8,0x20001ff0,0x100f0fe0,0x00000000,0x78000000,0xf0000000,0x47e00000,0x00000000,0x10000fe0,0x207f87f0,0x6fa11ff8,0xa0413ffc,0x7ffbfffe};
        const uint32_t walls_horizontal[31] = {0x7fffffdc,0x3ffd7b94,0x1ff9adaa,0x0ff0dc14,0x07e1e7a2,0x0003ec14,0x0001c62a,0x00000dd6,0x000066a8,0x0000fc50,0x0001d6ec,0x1fe18dfe,0x1ff19bec,0x3ff80000,0x7ffcc000,0x2adf8000,0x2bcd8000,0x6b59c000,0x6b6de000,0x3bf8f000,0x6bfc7be7,0x0a003fef,0x3a001f1e,0x2a003388,0x4a007fc8,0x2a00f368,0x4bfd2a98,0x362ad560,0x0b552a90,0x0c2a1508,0x1e7f3f9e};
        const uint8_t goal_x = 7;
        const uint8_t goal_y = 6;
    };


// 第35回全日本大会 2014 ハーフエキスパート決勝
    class AllJapan2014Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x7fff7e7f,0x1fd8fe3e,0x0f81201c,0x1f025008,0x00008820,0x00315470,0x0063eaf8,0x00c405fc,0x00020ab8,0x00040540,0x00020eb0,0x1fc41d48,0x3fe20090,0x7ff5fd28,0x4f0e3f44,0x460003c2,0x5c340062,0xb2c80024,0x55440078,0xa2bc00f0,0x5549bfa8,0xae850050,0x50790028,0xabf080f0,0x50013f00,0xac00fe00,0x5ff018e0,0x27c70c00,0x43800000,0xf1c07800,0x7f7fbffe};
        const uint32_t walls_horizontal[31] = {0x7ffefffc,0x3ffd7ff8,0x7ffbbe70,0x2a07c420,0x5301da08,0x2800b41c,0x50e66b3e,0x3f0f557f,0x7c1e6afe,0x3c0f3540,0x200f82a0,0x10150750,0x380a8fa8,0x1dff0054,0x7bdf8028,0x3c0fc01a,0x579fbfb4,0x57f5006c,0x5f82fffc,0x5f6dbff8,0x382adf9c,0x70578f3c,0x0029c078,0x00d380f0,0x00ae0000,0x63400000,0x32df0000,0x353c8000,0x2a01cffc,0x14029ffe,0x08003ffc};
        const uint8_t goal_x = 25;
        const uint8_t goal_y = 6;
    };


// 第36回全日本大会 2015 ハーフエキスパート決勝
    class AllJapan2015Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x7fffdffb,0x3eff8024,0x017f001e,0x0e803fcc,0x17001f80,0x00000006,0x0000800f,0x0700401e,0x2a80801f,0x7500001e,0x7a800003,0x3f7f1f80,0x00a8efc0,0x00547f86,0x002a3f0f,0x0015001e,0x000a0013,0x000d003e,0x0fea801b,0x7e7d0016,0x0cfc203c,0x07761030,0x0221e810,0x0017d418,0x209ea228,0x1fec4d50,0x0fd802a0,0x000c1940,0x00063fbe,0x000ad97e,0x3fdf7ffc};
        const uint32_t walls_horizontal[31] = {0x4ff0303c,0x0ba01812,0x0ccb3c20,0x0c007876,0x0a00fef8,0x15dffffc,0x0bfbcfea,0x17ff8004,0x0a00000a,0x31000004,0x0a80000a,0x05400004,0x0ea01fea,0x1d5f8bfe,0x3a2fce7c,0x5e55ed58,0x1e3aa550,0x2c855aa8,0x00c2aaa8,0x30cd4550,0x3c07a550,0x7b005aa8,0x01c02aa8,0x00d01474,0x01200a08,0x03800508,0x01c002e4,0x7ae7f5f8,0x3dffe0c8,0x7ce7f1fc,0x7ffffbfe};
        const uint8_t goal_x = 6;
        const uint8_t goal_y = 25;
    };


// 第37回全日本大会 2016 ハーフエキスパート決勝
    class AllJapan2016Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x87ffff9d,0x33ffffc0,0x79f94460,0x3efe12d6,0x1ff7e429,0x70fe6954,0x000396a8,0x3f07094c,0x01bbf6b0,0x00165128,0x181f2954,0x00021628,0x0084eb20,0x02005458,0x0036a82c,0x41e15754,0x3f4aad58,0xfc8592ac,0x7e8af750,0xf757c0a0,0xa8abb060,0xa84f7ff0,0x54b03f80,0x55481fc8,0xaa8724f4,0xadf8ff78,0x5a0076c6,0x5400eda2,0xa8007bf2,0x5000bc02,0x20017ffc};
        const uint32_t walls_horizontal[31] = {0x87ffffcc,0x35ffffee,0x7af82a10,0x3d7d1560,0x4992e8de,0x284d1aa1,0x4002f750,0x263dfadc,0x41020534,0x30002ae4,0x003ed558,0x0a0aaba4,0x01115458,0x0086b034,0x41e15dc8,0x3f42ae58,0xfc857000,0x7e8a200c,0xf755bbc0,0xa8abf030,0xa8573c00,0x549e7fc0,0x5560cb80,0xaa907cc0,0xad0ebe68,0x5bf1dfc4,0x5400fea6,0xa801f382,0x5000fea2,0x20017f92,0x4002fffc};
        const uint8_t goal_x = 3;
        const uint8_t goal_y = 2;
    };


// 第38回全日本大会 2017 ハーフエキスパート決勝
    class AllJapan2017Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x0000006f,0xc0701cc7,0x7ffffffe,0xfe5380e0,0x00280000,0x00500000,0x0f2b80ef,0x7ffffffe,0xea701d57,0x140000a0,0x0a000150,0xd4701ca7,0x1fffbffe,0x0ff394e5,0x0038140a,0x00701405,0x3e7b94ea,0x67ff9fde,0xc0701c07,0x00000000,0x00000000,0xc0701c07,0x0ff3ebe7,0x0e0394e4,0x00002a02,0x00001404,0x0e03aae2,0x7fff7cfa,0xca701cea,0x110000ea,0x0a0000ea};
        const uint32_t walls_horizontal[31] = {0x65000038,0x00000070,0x05380e3a,0x7ffefb74,0x0755c076,0x00280000,0x00540000,0x0729c07b,0x7f5fbeb6,0xe0394e7b,0x000080f0,0x00014078,0xe0388ef3,0xabfffef6,0x5729cf75,0xa0281e05,0x50540f05,0xa757ffff,0x7ffddef6,0xeab00e03,0x05000000,0x0a000000,0xe539ce73,0xebf9cf6d,0x073dca72,0x70780405,0xe03c0a02,0x077fd475,0x7ffceae6,0xe7f9fe73,0x0f83e0e0};
        const uint8_t goal_x = 20;
        const uint8_t goal_y = 21;
    };


// 第39回全日本大会 2018 ハーフエキスパート決勝
    class AllJapan2018Final_HF {
      public:
        const uint32_t walls_vertical[31] = {0x7fffdff9,0xe1ffbffa,0x7db7fffc,0x3f1cfb1e,0x3e007e3c,0x007c3e2c,0x003e0038,0x005c0000,0x00380000,0x3e500038,0x7db03e3c,0xdfbe4338,0x0fdfc7f0,0x07efbee0,0x0007cc40,0x00038df0,0x00010ae0,0x000107c0,0x07c3400a,0x0ff7e004,0x1f7ff004,0x3d3fc000,0x1c1f07c0,0x0c000fe0,0x04801ff0,0x04003fe8,0x0600001c,0x0f1f1e1e,0x3f1fbf02,0x1f7fbf82,0x7ffffffa};
        const uint32_t walls_horizontal[31] = {0x6ffbffec,0x07efffc2,0x73f7f384,0x61fbe000,0x60fec000,0x6c7c87f8,0x38009ff0,0x1400cf78,0x0c0347c0,0x00008000,0x007c3800,0x00ff8000,0x0d6f8000,0x3be7dfc0,0x5f871fe6,0x0f820ff0,0x00020ff8,0x00070f30,0x000f8e00,0x001fcc00,0x0f87e030,0x1fc7d01c,0x3ee00ff8,0x0fa007f0,0x07c003e0,0x02000002,0x0207c004,0x070fe002,0x0f9ff000,0x3ffffbe0,0x7ffbe7f8};
        const uint8_t goal_x = 12;
        const uint8_t goal_y = 12;
    };


// テスト迷路1
    class Test1_HF {
      public:
        const uint32_t walls_vertical[31] = {0x000000ff,0x00000000,0xffefffff,0x00000000,0xbfffffff,0x00000000,0x01000000,0xf7ffffff,0x00000000,0x00000000,0xdbbffefb,0x00400000,0x00000000,0xbebfffef,0x00000000,0xbbbffeef,0x00000000,0x00048000,0xbbfffff7,0x00000220,0x00800000,0x00000000,0xdfeeffef,0x00040000,0x04000000,0xffeffff7,0x00000008,0xbdeedfff,0x00000000,0x00000000,0x00000000};
        const uint32_t walls_horizontal[31] = {0x00000000,0x00000000,0x00000020,0x00800000,0x00080000,0x00400020,0x7eddfdfa,0x00080000,0x00000250,0x00000000,0xbefdfdfe,0x00028000,0x00020000,0x00020000,0x00000000,0xfeffedfe,0x00000000,0x01000000,0xfbddedbe,0x00000020,0x00000000,0x00000000,0x00200000,0xdffdffad,0x00000040,0x00000000,0x01000000,0xfb777f6d,0x00000000,0x00000000,0x00000000};
        const uint8_t goal_x = 31;
        const uint8_t goal_y = 31;
    };

}
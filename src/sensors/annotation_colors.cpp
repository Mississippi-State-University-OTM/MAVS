/*
MIT License

Copyright (c) 2024 Mississippi State University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <sensors/annotation_colors.h>

namespace mavs {
namespace sensor {

AnnotationColors::AnnotationColors() {
	annotation_colors_.resize(256);
	annotation_colors_[0] = glm::vec3(0, 84, 61);    // deepgreen
	annotation_colors_[1] = glm::vec3(0, 42, 41);    // verydarkbluishgreen
	annotation_colors_[2] = glm::vec3(0, 46, 59);    // verydarkgreenishblue
	annotation_colors_[3] = glm::vec3(0, 73, 88);    // darkgreenishblue
	annotation_colors_[4] = glm::vec3(0, 75, 73);    // darkbluishgreen
	annotation_colors_[5] = glm::vec3(0, 68, 63);    // deepbluishgreen
	annotation_colors_[6] = glm::vec3(27, 77, 62);    // darkgreen
	annotation_colors_[7] = glm::vec3(43, 61, 38);    // darkolivegreen
	annotation_colors_[8] = glm::vec3(23, 54, 32);    // verydarkyellowishgreen
	annotation_colors_[9] = glm::vec3(0, 49, 24);    // verydeepyellowishgreen
	annotation_colors_[10] = glm::vec3(28, 53, 45);    // verydarkgreen
	annotation_colors_[11] = glm::vec3(54, 53, 39);    // darkgrayisholive
	annotation_colors_[12] = glm::vec3(49, 54, 43);    // darkgrayisholivegreen
	annotation_colors_[13] = glm::vec3(62, 50, 44);    // darkgrayishbrown
	annotation_colors_[14] = glm::vec3(37, 36, 29);    // oliveblack
	annotation_colors_[15] = glm::vec3(40, 32, 28);    // brownishblack
	annotation_colors_[16] = glm::vec3(59, 49, 33);    // darkolivebrown
	annotation_colors_[17] = glm::vec3(64, 61, 33);    // darkolive
	annotation_colors_[18] = glm::vec3(72, 60, 50);    // darkgrayishyellowishbrown
	annotation_colors_[19] = glm::vec3(58, 75, 71);    // darkgrayishgreen
	annotation_colors_[20] = glm::vec3(54, 69, 79);    // darkgrayishblue
	annotation_colors_[21] = glm::vec3(26, 36, 33);    // blackishgreen
	annotation_colors_[22] = glm::vec3(30, 35, 33);    // greenishblack
	annotation_colors_[23] = glm::vec3(32, 36, 40);    // bluishblack
	annotation_colors_[24] = glm::vec3(32, 40, 48);    // blackishblue
	annotation_colors_[25] = glm::vec3(34, 34, 34);    // black
	annotation_colors_[26] = glm::vec3(36, 33, 36);    // purplishblack
	annotation_colors_[27] = glm::vec3(40, 32, 34);    // reddishblack
	annotation_colors_[28] = glm::vec3(0, 48, 78);    // darkblue
	annotation_colors_[29] = glm::vec3(0, 65, 106);    // deepblue
	annotation_colors_[30] = glm::vec3(76, 81, 109);    // grayishpurplishblue
	annotation_colors_[31] = glm::vec3(0, 103, 165);    // strongblue
	annotation_colors_[32] = glm::vec3(0, 119, 145);    // stronggreenishblue
	annotation_colors_[33] = glm::vec3(54, 117, 136);    // moderategreenishblue
	annotation_colors_[34] = glm::vec3(46, 132, 149);    // deepgreenishblue
	annotation_colors_[35] = glm::vec3(0, 133, 161);    // vividgreenishblue
	annotation_colors_[36] = glm::vec3(67, 107, 149);    // moderateblue
	annotation_colors_[37] = glm::vec3(94, 113, 106);    // grayishgreen
	annotation_colors_[38] = glm::vec3(83, 104, 120);    // grayishblue
	annotation_colors_[39] = glm::vec3(81, 88, 94);    // darkbluishgray
	annotation_colors_[40] = glm::vec3(85, 85, 85);    // darkgray
	annotation_colors_[41] = glm::vec3(93, 85, 91);    // darkpurplishgray
	annotation_colors_[42] = glm::vec3(78, 87, 85);    // darkgreenishgray
	annotation_colors_[43] = glm::vec3(91, 80, 79);    // brownishgray
	annotation_colors_[44] = glm::vec3(92, 80, 79);    // darkreddishgray
	annotation_colors_[45] = glm::vec3(87, 85, 76);    // olivegray
	annotation_colors_[46] = glm::vec3(99, 81, 71);    // grayishbrown
	annotation_colors_[47] = glm::vec3(81, 87, 68);    // grayisholivegreen
	annotation_colors_[48] = glm::vec3(91, 88, 66);    // grayisholive
	annotation_colors_[49] = glm::vec3(126, 109, 90);    // grayishyellowishbrown
	annotation_colors_[50] = glm::vec3(53, 94, 59);    // darkyellowishgreen
	annotation_colors_[51] = glm::vec3(49, 120, 115);    // moderatebluishgreen
	annotation_colors_[52] = glm::vec3(0, 136, 130);    // vividbluishgreen
	annotation_colors_[53] = glm::vec3(0, 122, 116);    // strongbluishgreen
	annotation_colors_[54] = glm::vec3(59, 120, 97);    // moderategreen
	annotation_colors_[55] = glm::vec3(0, 121, 89);    // stronggreen
	annotation_colors_[56] = glm::vec3(0, 161, 194);    // vividblue
	annotation_colors_[57] = glm::vec3(35, 158, 186);    // brilliantgreenishblue
	annotation_colors_[58] = glm::vec3(73, 151, 208);    // brilliantblue
	annotation_colors_[59] = glm::vec3(112, 163, 204);    // lightblue
	annotation_colors_[60] = glm::vec3(102, 170, 188);    // lightgreenishblue
	annotation_colors_[61] = glm::vec3(161, 202, 241);    // verylightblue
	annotation_colors_[62] = glm::vec3(179, 188, 226);    // verylightpurplishblue
	annotation_colors_[63] = glm::vec3(150, 222, 209);    // verylightbluishgreen
	annotation_colors_[64] = glm::vec3(142, 209, 178);    // verylightgreen
	annotation_colors_[65] = glm::vec3(199, 230, 215);    // verypalegreen
	annotation_colors_[66] = glm::vec3(223, 237, 232);    // greenishwhite
	annotation_colors_[67] = glm::vec3(232, 227, 229);    // purplishwhite
	annotation_colors_[68] = glm::vec3(233, 233, 237);    // bluishwhite
	annotation_colors_[69] = glm::vec3(242, 243, 244);    // white
	annotation_colors_[70] = glm::vec3(240, 234, 214);    // yellowishwhite
	annotation_colors_[71] = glm::vec3(236, 213, 197);    // paleyellowishpink
	annotation_colors_[72] = glm::vec3(234, 227, 225);    // pinkishwhite
	annotation_colors_[73] = glm::vec3(178, 190, 181);    // lightgreenishgray
	annotation_colors_[74] = glm::vec3(191, 185, 189);    // lightpurplishgray
	annotation_colors_[75] = glm::vec3(192, 200, 225);    // verypalepurplishblue
	annotation_colors_[76] = glm::vec3(196, 195, 221);    // verypaleviolet
	annotation_colors_[77] = glm::vec3(188, 212, 230);    // verypaleblue
	annotation_colors_[78] = glm::vec3(180, 188, 192);    // lightbluishgray
	annotation_colors_[79] = glm::vec3(156, 209, 220);    // verylightgreenishblue
	annotation_colors_[80] = glm::vec3(141, 163, 153);    // palegreen
	annotation_colors_[81] = glm::vec3(145, 163, 176);    // paleblue
	annotation_colors_[82] = glm::vec3(193, 182, 179);    // pinkishgray
	annotation_colors_[83] = glm::vec3(185, 184, 181);    // lightgray
	annotation_colors_[84] = glm::vec3(194, 172, 153);    // brownishpink
	annotation_colors_[85] = glm::vec3(191, 184, 165);    // yellowishgray
	annotation_colors_[86] = glm::vec3(174, 155, 130);    // lightgrayishyellowishbrown
	annotation_colors_[87] = glm::vec3(143, 151, 121);    // grayishyellowgreen
	annotation_colors_[88] = glm::vec3(138, 135, 118);    // lightolivegray
	annotation_colors_[89] = glm::vec3(140, 135, 103);    // lightgrayisholive
	annotation_colors_[90] = glm::vec3(149, 128, 112);    // lightgrayishbrown
	annotation_colors_[91] = glm::vec3(142, 130, 121);    // lightbrownishgray
	annotation_colors_[92] = glm::vec3(143, 129, 127);    // reddishgray
	annotation_colors_[93] = glm::vec3(139, 133, 137);    // purplishgray
	annotation_colors_[94] = glm::vec3(129, 135, 139);    // bluishgray
	annotation_colors_[95] = glm::vec3(132, 132, 130);    // mediumgray
	annotation_colors_[96] = glm::vec3(140, 146, 172);    // palepurplishblue
	annotation_colors_[97] = glm::vec3(125, 137, 132);    // greenishgray
	annotation_colors_[98] = glm::vec3(106, 171, 142);    // lightgreen
	annotation_colors_[99] = glm::vec3(102, 173, 164);    // lightbluishgreen
	annotation_colors_[100] = glm::vec3(0, 166, 147);    // brilliantbluishgreen
	annotation_colors_[101] = glm::vec3(62, 180, 137);    // brilliantgreen
	annotation_colors_[102] = glm::vec3(68, 148, 74);    // strongyellowishgreen
	annotation_colors_[103] = glm::vec3(39, 166, 76);    // vividyellowishgreen
	annotation_colors_[104] = glm::vec3(147, 197, 146);    // lightyellowishgreen
	annotation_colors_[105] = glm::vec3(103, 146, 103);    // moderateyellowishgreen
	annotation_colors_[106] = glm::vec3(138, 154, 91);    // moderateyellowgreen
	annotation_colors_[107] = glm::vec3(152, 148, 62);    // darkgreenishyellow
	annotation_colors_[108] = glm::vec3(171, 145, 68);    // darkyellow
	annotation_colors_[109] = glm::vec3(161, 143, 96);    // darkgrayishyellow
	annotation_colors_[110] = glm::vec3(194, 178, 128);    // grayishyellow
	annotation_colors_[111] = glm::vec3(185, 181, 125);    // grayishgreenishyellow
	annotation_colors_[112] = glm::vec3(201, 174, 93);    // moderateyellow
	annotation_colors_[113] = glm::vec3(185, 180, 89);    // moderategreenishyellow
	annotation_colors_[114] = glm::vec3(201, 220, 137);    // lightyellowgreen
	annotation_colors_[115] = glm::vec3(250, 214, 165);    // paleorangeyellow
	annotation_colors_[116] = glm::vec3(218, 223, 183);    // paleyellowgreen
	annotation_colors_[117] = glm::vec3(243, 229, 171);    // paleyellow
	annotation_colors_[118] = glm::vec3(248, 222, 126);    // lightyellow
	annotation_colors_[119] = glm::vec3(235, 232, 164);    // palegreenishyellow
	annotation_colors_[120] = glm::vec3(182, 229, 175);    // verylightyellowishgreen
	annotation_colors_[121] = glm::vec3(131, 211, 125);    // brilliantyellowishgreen
	annotation_colors_[122] = glm::vec3(189, 218, 87);    // brilliantyellowgreen
	annotation_colors_[123] = glm::vec3(233, 228, 80);    // brilliantgreenishyellow
	annotation_colors_[124] = glm::vec3(234, 230, 121);    // lightgreenishyellow
	annotation_colors_[125] = glm::vec3(250, 218, 94);    // brilliantyellow
	annotation_colors_[126] = glm::vec3(243, 195, 0);    // vividyellow
	annotation_colors_[127] = glm::vec3(220, 211, 0);    // vividgreenishyellow
	annotation_colors_[128] = glm::vec3(190, 183, 46);    // stronggreenishyellow
	annotation_colors_[129] = glm::vec3(212, 175, 55);    // strongyellow
	annotation_colors_[130] = glm::vec3(175, 141, 19);    // deepyellow
	annotation_colors_[131] = glm::vec3(155, 148, 0);    // deepgreenishyellow
	annotation_colors_[132] = glm::vec3(141, 182, 0);    // vividyellowgreen
	annotation_colors_[133] = glm::vec3(126, 159, 46);    // strongyellowgreen
	annotation_colors_[134] = glm::vec3(0, 136, 86);    // vividgreen
	annotation_colors_[135] = glm::vec3(70, 113, 41);    // deepyellowgreen
	annotation_colors_[136] = glm::vec3(0, 98, 45);    // deepyellowishgreen
	annotation_colors_[137] = glm::vec3(74, 93, 35);    // moderateolivegreen
	annotation_colors_[138] = glm::vec3(102, 93, 30);    // moderateolive
	annotation_colors_[139] = glm::vec3(134, 126, 54);    // lightolive
	annotation_colors_[140] = glm::vec3(108, 84, 30);    // moderateolivebrown
	annotation_colors_[141] = glm::vec3(35, 47, 0);    // deepolivegreen
	annotation_colors_[142] = glm::vec3(64, 79, 0);    // strongolivegreen
	annotation_colors_[143] = glm::vec3(150, 113, 23);    // lightolivebrown
	annotation_colors_[144] = glm::vec3(190, 101, 22);    // deeporange
	annotation_colors_[145] = glm::vec3(86, 7, 12);    // deepreddishbrown
	annotation_colors_[146] = glm::vec3(89, 51, 25);    // deepbrown
	annotation_colors_[147] = glm::vec3(101, 69, 34);    // deepyellowishbrown
	annotation_colors_[148] = glm::vec3(111, 78, 55);    // moderatebrown
	annotation_colors_[149] = glm::vec3(130, 102, 68);    // moderateyellowishbrown
	annotation_colors_[150] = glm::vec3(153, 101, 21);    // strongyellowishbrown
	annotation_colors_[151] = glm::vec3(128, 70, 27);    // strongbrown
	annotation_colors_[152] = glm::vec3(174, 105, 56);    // brownishorange
	annotation_colors_[153] = glm::vec3(158, 71, 50);    // darkreddishorange
	annotation_colors_[154] = glm::vec3(188, 63, 74);    // strongred
	annotation_colors_[155] = glm::vec3(170, 56, 30);    // deepreddishorange
	annotation_colors_[156] = glm::vec3(190, 0, 50);    // vividred
	annotation_colors_[157] = glm::vec3(136, 45, 23);    // strongreddishbrown
	annotation_colors_[158] = glm::vec3(226, 88, 34);    // vividreddishorange
	annotation_colors_[159] = glm::vec3(230, 103, 33);    // deepyellowishpink
	annotation_colors_[160] = glm::vec3(201, 133, 0);    // deeporangeyellow
	annotation_colors_[161] = glm::vec3(234, 162, 33);    // strongorangeyellow
	annotation_colors_[162] = glm::vec3(246, 166, 0);    // vividorangeyellow
	annotation_colors_[163] = glm::vec3(237, 135, 45);    // strongorange
	annotation_colors_[164] = glm::vec3(253, 148, 63);    // brilliantorange
	annotation_colors_[165] = glm::vec3(243, 132, 0);    // vividorange
	annotation_colors_[166] = glm::vec3(255, 193, 79);    // brilliantorangeyellow
	annotation_colors_[167] = glm::vec3(251, 201, 127);    // lightorangeyellow
	annotation_colors_[168] = glm::vec3(250, 181, 127);    // lightorange
	annotation_colors_[169] = glm::vec3(255, 183, 165);    // vividyellowishpink
	annotation_colors_[170] = glm::vec3(249, 147, 121);    // strongyellowishpink
	annotation_colors_[171] = glm::vec3(217, 144, 88);    // moderateorange
	annotation_colors_[172] = glm::vec3(227, 168, 87);    // moderateorangeyellow
	annotation_colors_[173] = glm::vec3(193, 154, 107);    // lightyellowishbrown
	annotation_colors_[174] = glm::vec3(166, 123, 91);    // lightbrown
	annotation_colors_[175] = glm::vec3(190, 138, 61);    // darkorangeyellow
	annotation_colors_[176] = glm::vec3(203, 109, 81);    // moderatereddishorange
	annotation_colors_[177] = glm::vec3(180, 116, 94);    // grayishreddishorange
	annotation_colors_[178] = glm::vec3(217, 96, 59);    // strongreddishorange
	annotation_colors_[179] = glm::vec3(222, 111, 161);    // deeppurplishpink
	annotation_colors_[180] = glm::vec3(230, 143, 172);    // strongpurplishpink
	annotation_colors_[181] = glm::vec3(228, 113, 122);    // deeppink
	annotation_colors_[182] = glm::vec3(196, 131, 121);    // darkyellowishpink
	annotation_colors_[183] = glm::vec3(192, 128, 129);    // darkpink
	annotation_colors_[184] = glm::vec3(183, 132, 167);    // lightreddishpurple
	annotation_colors_[185] = glm::vec3(193, 126, 145);    // darkpurplishpink
	annotation_colors_[186] = glm::vec3(170, 138, 158);    // palereddishpurple
	annotation_colors_[187] = glm::vec3(150, 144, 171);    // paleviolet
	annotation_colors_[188] = glm::vec3(151, 127, 115);    // lightgrayishreddishbrown
	annotation_colors_[189] = glm::vec3(168, 124, 109);    // lightreddishbrown
	annotation_colors_[190] = glm::vec3(175, 134, 142);    // lightgrayishpurplishred
	annotation_colors_[191] = glm::vec3(173, 136, 132);    // lightgrayishred
	annotation_colors_[192] = glm::vec3(217, 166, 169);    // moderateyellowishpink
	annotation_colors_[193] = glm::vec3(222, 165, 164);    // moderatepink
	annotation_colors_[194] = glm::vec3(199, 173, 163);    // grayishyellowishpink
	annotation_colors_[195] = glm::vec3(196, 174, 173);    // grayishpink
	annotation_colors_[196] = glm::vec3(195, 166, 177);    // grayishpurplishpink
	annotation_colors_[197] = glm::vec3(170, 152, 169);    // palepurple
	annotation_colors_[198] = glm::vec3(213, 151, 174);    // moderatepurplishpink
	annotation_colors_[199] = glm::vec3(234, 147, 153);    // strongpink
	annotation_colors_[200] = glm::vec3(255, 181, 186);    // vividpink
	annotation_colors_[201] = glm::vec3(255, 200, 214);    // brilliantpurplishpink
	annotation_colors_[202] = glm::vec3(239, 187, 204);    // lightpurplishpink
	annotation_colors_[203] = glm::vec3(213, 186, 219);    // verylightpurple
	annotation_colors_[204] = glm::vec3(214, 202, 221);    // verypalepurple
	annotation_colors_[205] = glm::vec3(232, 204, 215);    // palepurplishpink
	annotation_colors_[206] = glm::vec3(244, 194, 194);    // lightyellowishpink
	annotation_colors_[207] = glm::vec3(234, 216, 215);    // palepink
	annotation_colors_[208] = glm::vec3(249, 204, 202);    // lightpink
	annotation_colors_[209] = glm::vec3(220, 208, 255);    // verylightviolet
	annotation_colors_[210] = glm::vec3(211, 153, 230);    // brilliantpurple
	annotation_colors_[211] = glm::vec3(182, 149, 192);    // lightpurple
	annotation_colors_[212] = glm::vec3(135, 145, 191);    // lightpurplishblue
	annotation_colors_[213] = glm::vec3(140, 130, 182);    // lightviolet
	annotation_colors_[214] = glm::vec3(144, 101, 202);    // vividviolet
	annotation_colors_[215] = glm::vec3(154, 78, 174);    // vividpurple
	annotation_colors_[216] = glm::vec3(158, 79, 136);    // strongreddishpurple
	annotation_colors_[217] = glm::vec3(206, 70, 118);    // vividpurplishred
	annotation_colors_[218] = glm::vec3(171, 78, 82);    // moderatered
	annotation_colors_[219] = glm::vec3(168, 81, 110);    // moderatepurplishred
	annotation_colors_[220] = glm::vec3(179, 68, 108);    // strongpurplishred
	annotation_colors_[221] = glm::vec3(145, 92, 131);    // moderatereddishpurple
	annotation_colors_[222] = glm::vec3(145, 95, 109);    // grayishpurplishred
	annotation_colors_[223] = glm::vec3(121, 68, 59);    // moderatereddishbrown
	annotation_colors_[224] = glm::vec3(144, 93, 93);    // grayishred
	annotation_colors_[225] = glm::vec3(103, 76, 71);    // grayishreddishbrown
	annotation_colors_[226] = glm::vec3(121, 104, 120);    // grayishpurple
	annotation_colors_[227] = glm::vec3(131, 100, 121);    // grayishreddishpurple
	annotation_colors_[228] = glm::vec3(108, 121, 184);    // brilliantpurplishblue
	annotation_colors_[229] = glm::vec3(126, 115, 184);    // brilliantviolet
	annotation_colors_[230] = glm::vec3(134, 96, 142);    // moderatepurple
	annotation_colors_[231] = glm::vec3(135, 86, 146);    // strongpurple
	annotation_colors_[232] = glm::vec3(96, 78, 151);    // strongviolet
	annotation_colors_[233] = glm::vec3(84, 90, 167);    // strongpurplishblue
	annotation_colors_[234] = glm::vec3(78, 81, 128);    // moderatepurplishblue
	annotation_colors_[235] = glm::vec3(96, 78, 129);    // moderateviolet
	annotation_colors_[236] = glm::vec3(85, 76, 105);    // grayishviolet
	annotation_colors_[237] = glm::vec3(96, 47, 107);    // deeppurple
	annotation_colors_[238] = glm::vec3(84, 25, 78);    // verydeepreddishpurple
	annotation_colors_[239] = glm::vec3(64, 26, 76);    // verydeeppurple
	annotation_colors_[240] = glm::vec3(50, 23, 77);    // deepviolet
	annotation_colors_[241] = glm::vec3(48, 38, 122);    // vividpurplishblue
	annotation_colors_[242] = glm::vec3(39, 36, 88);    // deeppurplishblue
	annotation_colors_[243] = glm::vec3(47, 33, 64);    // darkviolet
	annotation_colors_[244] = glm::vec3(37, 36, 64);    // darkpurplishblue
	annotation_colors_[245] = glm::vec3(41, 30, 41);    // blackishpurple
	annotation_colors_[246] = glm::vec3(46, 29, 33);    // blackishred
	annotation_colors_[247] = glm::vec3(56, 21, 44);    // verydarkpurplishred
	annotation_colors_[248] = glm::vec3(52, 23, 49);    // verydarkreddishpurple
	annotation_colors_[249] = glm::vec3(48, 25, 52);    // verydarkpurple
	annotation_colors_[250] = glm::vec3(93, 57, 84);    // darkreddishpurple
	annotation_colors_[251] = glm::vec3(86, 60, 92);    // darkpurple
	annotation_colors_[252] = glm::vec3(80, 64, 77);    // darkgrayishpurple
	annotation_colors_[253] = glm::vec3(84, 61, 63);    // darkgrayishred
	annotation_colors_[254] = glm::vec3(75, 54, 33);    // darkyellowishbrown
	annotation_colors_[255] = glm::vec3(66, 37, 24);    // darkbrown
}

glm::vec3 AnnotationColors::GetColor(int i) {
	return annotation_colors_[(i%annotation_colors_.size())];
}

void AnnotationColors::SetColor(int i, float r, float g, float b) {
	annotation_colors_[(i%annotation_colors_.size())] = glm::vec3(r, g, b);
}

void AnnotationColors::AddColor(float r, float g, float b) {
	annotation_colors_.push_back(glm::vec3(r,g,b));
}

} // namespace sensors
} // namespace mavs
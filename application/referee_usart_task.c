/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM闂傚倸鍊峰ù鍥х暦閻㈢ǹ绐楃€广儱鎷�?〒濠�?��閻戞ɑ鈷掗柣�?炴閵嗘帒顫濋敐鍛闁诲氦�??ú鏍礊婵犲洤绠栭柛鎾楀倹鍕�?��鑺ッˇ浠�?��濞嗘挻鈷掑ù锝�?��鐢埖銇�?��锝嗙鐎规洘鍔曡灃闁告侗鍘藉Σ顒勬⒑缂佹ǘ缂氱紒顕呭灦閹€斥�?濡繐缍婇弫鎰板川椤旇�?鍓垫俊鐐€х紞鈧柛瀣姍閸┾偓妞ゆ帒鍠�?��鎰版煙閸濄儱鍘撮柟�?惧仱閺佸啴鍩�?��掑�?鏁�?柕澶嗘櫅缁€瀣亜閺嶃劍鐨戞い锔芥緲椤啴濡堕崱�?���?��佸摜�?��柊锝夊春閳ь剚銇勯幒鎴敾閻庢�?���?
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

/**
 * Modified the "RM referee system data solve" into CV_data receiving and processing
 * Commented out the original referee_usart_task function and added the UART7_CommandRoute function
 */

#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"

#include <stdbool.h> // Include this header to use bool type

/**
 * @brief          single byte upacked
 * @param[in]      void
 * @retval         none
 */
/**
 * @brief          闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩顔瑰亾閸愵喖骞㈡俊鐐存�?濡炶棄�?ｆ�?瀣垫晣闁绘�?娓瑰Σ鎰版煟閻斿摜�??俊�?㈠暣閻涱喗绻濋崶銊モ偓鐑芥煕濠靛棗�?柨娑欑矊閳规垿�?��鑲╁姶闂佸憡鎸诲銊╁极椤曗偓閸ㄦ儳鐣烽崶銊︻啎闂備浇娉曢崳锕傚�?閿燂�??
 * @param[in]      void
 * @retval         none
 */
static void referee_unpack_fifo_data(void);

extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
char dma_buf[50];

// 闂傚倸鍊搁崐鐑芥嚄閸洖纾块柣銏⑶圭壕濠�?��閹�?劦鍤欑紒鈧径鎰€甸柨婵嗛�?ф彃鈹戦鎸庣彧闁靛洤瀚伴�?��楀箣濠�?���?��紓鍌欒兌婵灚绻涙繝鍥ц摕闁哄洨鍠庣�?鐐�?箾閹存瑥鐏い搴＄Ч濮婃椽宕崟�?婄�?婵炲瓨绮犳�?顏勵嚕婵犳�?鏅插璺好￠埡鍛厪濠㈣泛鐗嗛崝婊堟煟閵堝懎鐨瑼闂傚倸鍊搁崐宄懊归崶褜娴�?��濞炬櫆閸ゅ�?鏌ょ粙璺ㄤ粵婵炲懐濮�?��濠囧Χ閸屾�?��曢梻浣�?串缁�?���?��璺虹闁告稑鐡ㄩ幆鐐烘煕閿旇�?��紒杈ㄥ▕�?���?���?��瀣秷闂佺ǹ�?哥�?顓熺珶閺囥埄鏁囬柕蹇曞Х椤旀垿�?虹捄銊ユ珢闁瑰嚖鎷�
// DMA data buffer for reception
unsigned char rx_data[50];

/* 婵犵數濮烽弫鎼佸磻濞戙埄鏁�?い鎾跺枑閸欏繐霉閸忓吋缍戠痪�?�健閺岀喎鈻撻崹顔界亾缂備胶�?��Λ鍐蓟濞戞ǚ妲堟繛鍡樺姉缁�?��姊洪崨濠�?��闁挎洦浜濠氬Ω閳哄倸浜滈柣鐘叉处閻ｎ偊骞橀崜浣猴紲闁�?函缍嗘�?璺何熼埀顒勬⒑缁洘鏉归柛瀣�?��啴濡堕崱妤�?��闂佺厧鍟块悥濂�?春閵忋倕绫嶉柛顐ゅ枔閸欏�?姊虹紒妯烩拻闁�?簺鍊曢埢宥�?即閵忥紕鍘撻悷�?�?��瀹曟粌鈽夐�?鐘电枃闂佽宕橀�?鈧艾顦伴妵鍕箳閹存繍浠煎┑�?款潐閻擄繝�?婚妸銉㈡斀闁糕檧鏅滅瑧缂傚倷鑳舵慨�?��箾婵犲洤钃熼柡鍥╁枎缁剁偞绻涢幋娆忕仾�?ゅ�?绉瑰娲传閸曨剨绱炴繛瀛�?矤娴滎亜顕ｆ繝�?労闁告劏鏅涢鎾绘⒑閸涘﹦绠撻悗�?卞厴瀵娊顢橀姀鈥斥偓鐢告偡濞嗗繐�?悘蹇ｅ亞缁辨帡鎳滄担鍐�?闂佷�?��块崗姗€骞冮埡鍐╁珰闁圭�?��為悰鈺備繆閻愵亜鈧牠鎮уΔ鍐煓闁规崘顕уЧ鏌ユ煟濡偐甯涢柍閿�?灴閺屾稑鈹戦崱妤婁紑闁煎弶鐗犲娲川婵犲海鍔堕梺鎼炲劘閸斿秴鈻嶉�?銈嗏拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚�? */
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩鍐蹭罕闂佸搫娲㈤崹鍦不閻�?绠规繛锝庡�?��＄厧顩奸崨瀛樷拺闁告繂瀚弳娆撴煟濡も偓閿曨亜顕ｉ弻銉晢闁告洦鍓涢崢�?��偡濠婂�?绠炴鐐�??閺佹捇鏁撻敓锟�55a闂傚倸鍊搁崐鐑芥倿閿旈敮鍋撶�?��樻噽閻瑩鏌熷▎鈥崇湴閸旀垿宕�?��顒併亜閹烘垵鈧崵澹曟總鍛婄厵闁�?���?��崕鎰版倵濮樼偓瀚�2闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲鑸�?拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚� */
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩鍐蹭罕闂佸搫娲㈤崹鍦不閻�?櫕鍙忔俊鐐�?嚙娴滈箖�??▓鍨灕婵炲懏娲栧玻鑳疀濞戞鈺�?��閺囨浜鹃梺鍛婃煥閹�??��忓ú�?�?��閹兼�?���?��鍧�?⒑缁�?��鍎愰柛鏃�?��堥崚鎺�?晲�?跺娅㈤梺璺ㄥ櫐閹凤�?1闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲鑸�?拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚� */
/* ---婵犵數濮烽弫鎼佸磻濞戙埄鏁�?い鎾跺枑閸欏繐霉閸忓吋缍戠痪�?�健閺岀喎鈻撻崹顔界亾缂備胶�?��Λ鍐蓟濞戞ǚ妲堟繛鍡樺姉缁�?��姊洪崨濠�?��闁挎洦浜濠氬Ω閳哄倸浜滈梺鍛婃�?��冲爼�?��幘缁樺€�?���?��仢閺�?��鏌ｈ箛鏃囧妞ゆ洏鍎靛畷鐔碱敇濞戞ü澹曢梺鎸庣箓妤犲憡绂嶅┑瀣厽婵犲灚鎸剧弧鈧┑�?硷攻濡炶棄鐣烽妸锔剧瘈闁稿本鍑瑰ḿ鎾绘⒒娴ｇ儤鍤�?��哥喎娼￠弻濠囨晲婢跺﹥鐎梺鐟板⒔缁垶宕戦埡鍛€堕柣鎰祷濡�?�霉閻樻瑥鎳�??钘�?归敐鍕煓闁告繆娅ｇ槐鎺旀嫚閼碱剙顣哄銈嗘穿缂嶄線骞�?��褌娌柟顖嗗�?��參姊绘担鍛婂暈婵炶绠撳畷鎴﹀幢濞戞褔鏌ㄩ弴鐐测偓褰掑煕閹烘鐓曢悘鐐插⒔椤ｈ尙绱掗埀顒€鐣濋崟�?傚幈闁�?函缍嗛崑鍛焊娴煎瓨鐓欏〒�?仢婵倿鏌熼�?�幋鐎�?喛鍩�?��鏃堝箻閸忓懎顥氭繝娈�?��缁�?倻鎮锝囶洸闁荤喖鍋婇悢鍡涙煠閹间焦娑у┑顔兼川缁辨帡鍩€椤掑�?��搁柨鐕傛�??--- */
/* hex_Yaw闂傚倸鍊搁崐椋庣矆娴ｈ桨鐒婇柛娑欐綑缁狀垶鏌ㄩ悤鍌涘�??4闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲顒�?富闁靛牆�?��亸�?堟煕閵�?附銇濈€殿喗鐓￠、妤�??椤掑倸骞堟俊鐐�?��崝妤佹叏閹绢喖绀夋繝濠傚幘瑜版帗鍋愮紒娑氼攰閳ь剙娼￠弻鐔碱敊閹冨�?閻庢鍠楀ḿ娆擄綖濠靛鏁�?��姗嗗枛娴犳帒鈹戦悩鍨毄闁�?鍋涘玻�?��枎瀵邦偅绋戦埞鎴犫偓锝庡�?缁侊箓�?虹涵鍛涧闂傚�?瀚板鏌ュ�?娴ｅ湱鍘鹃梺璇″幗鐢帡宕濋妶澶�?厓缂備焦蓱閳锋帞绱掓潏銊ユ诞闁糕晛瀚板畷�?�?��﹂幋鐐版唉EE 754闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢妶鍥╃厠闂佸搫顦伴崵�?洪鍕�?��熆鐠虹尨�?��俊�?㈠槻椤啴濡堕崱娆忊拡闂佺ǹ�?�?��銈呯暦閺囩儐鍚�?璺�?��閸橀亶�?�?��鍕偍闁告柨鏈粋�?�?��閸涱垳锛滈梺�?���?��伴柍褜鍓ㄧ紞鍡涘礈濞戞艾顥氶柛蹇曨儠娴滄粓鏌￠崘銊﹀�?ゃ儱顦甸弻娑㈠�?鐠囨祴鍋撳┑瀣畺婵°倕鎳忛崑锟犳煛�?跺顕滄慨锝嗗�?�?��宕惰濡偓闂佸搫琚崝鎴﹀箖閵忋倕绀堝ù锝堟閸橆垶�?绘担绛�?殐闁哥姵顨婇幃鐤樄妤犵偛顦甸獮姗€顢欓懖鈺婃Ч婵＄偑鍊栫�?���?��閿熺姴鐤柍�?鍓熷濠�?��濞嗘垵濡介柣搴ｇ懗閸パ咁槰婵犵數�?��懝鍓х不椤�?��鐓ラ柣鏇炲€�?��氾拷 */
/* hex_Pitch闂傚倸鍊搁崐椋庣矆娴ｈ桨鐒婇柛娑欐綑缁狀垶鏌ㄩ悤鍌涘�??4闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲顒�?富闁靛牆�?��亸�?堟煕閵�?附銇濈€殿喗鐓￠、妤�??椤掑倸骞堟俊鐐�?��崝妤佹叏閹绢喖绀夋繝濠傚幘瑜版帗鍋愮紒娑氼攰閳ь剙娼￠弻鐔碱敊閹冨�?閻庢鍠楀ḿ娆擄綖濠靛鏁�?��姗嗗枛娴犳帒鈹戦悩鍨毄闁�?鍋涘玻�?��枎瀵邦偅绋戦埞鎴犫偓锝庡�?缁侊箓�?虹涵鍛涧闂傚�?瀚板鏌ュ�?娴ｅ湱鍘鹃梺璇″幗鐢帡宕濋妶澶�?厓缂備焦蓱閳锋帞绱掓潏銊ユ诞闁糕晛瀚板畷�?�?��﹂幋鐐版唉EE 754闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢妶鍥╃厠闂佸搫顦伴崵�?洪鍕�?��熆鐠虹尨�?��俊�?㈠槻椤啴濡堕崱娆忊拡闂佺ǹ�?�?��銈呯暦閺囩儐鍚�?璺�?��閸橀亶�?�?��鍕偍闁告柨鏈粋�?�?��閸涱垳锛滈梺�?���?��伴柍褜鍓ㄧ紞鍡涘礈濞戞艾顥氶柛蹇曨儠娴滄粓鏌￠崘銊﹀�?ゃ儱顦甸弻娑㈠�?鐠囨祴鍋撳┑瀣畺婵°倕鎳忛崑锟犳煛�?跺顕滄慨锝嗗�?�?��宕惰濡偓闂佸搫琚崝鎴﹀箖閵忋倕绀堝ù锝堟閸橆垶�?绘担绛�?殐闁哥姵顨婇幃鐤樄妤犵偛顦甸獮姗€顢欓懖鈺婃Ч婵＄偑鍊栫�?���?��閿熺姴鐤柍�?鍓熷濠�?��濞嗘垵濡介柣搴ｇ懗閸パ咁槰婵犵數�?��懝鍓х不椤�?��鐓ラ柣鏇炲€�?��氾拷 */
/* ---婵犵數濮烽弫鎼佸磻濞戙埄鏁�?い鎾跺枑閸欏繐霉閸忓吋缍戠痪�?�健閺岀喎鈻撻崹顔界亾缂備胶�?��Λ鍐蓟濞戞ḿ鏆嗛柍�?鍓熷畷鎴濃�?閵忕姷鐓戞繝鐢靛Т濞�?箓鎮￠妷鈺傜厽闁哄倸鐏濋。鎶芥煟椤撶喓鎳呴柍褜鍓氶鏍窗閺嶎厽鍊舵慨�?嗗墻閸ゆ鏌涢弴銊ュ箰闁稿鎹囬�?��償閳ヨ尙鍑归梻浣虹帛缁绘劗鎹㈠┑鍡╂綎婵炲樊浜滃婵嗏攽閻樻彃鈧悂锝炲畝鍕拺闁荤喐婢橀弳閬嶆⒑鐢喚绉€殿噮鍋婇�?��肩磼濡�??熼梻浣芥硶閸ｏ箓骞忛敓锟�??--- */
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炴牠顢曢妶鍥╃�?��佸憡娲﹂崢钘夌暦閸欏绡�?��傚牊渚�?崕鎰版煕鐎ｎ倖鎴犳崲濞戙垹骞㈡俊顖濐嚙绾板秴鈹戦敍鍕户闁哄拋鍋婇崺鈧い鎺嗗亾闁告ɑ�?��畷鎴﹀箻缂佹�?��敻鏌ㄥ┑鍡涱€楀�?鍗抽弻锝�??閾忣偄�?�?1闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲鑸�?拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚� */
/* 闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩鍐蹭罕闂佸搫娲㈤崹鍦不閻�?绠规繛锝庡�?��′粙鏌涚�?��亜�?柕鍥у楠炴帡骞�?��鎰�?��備�?鎼幏瀣礈閻旂厧钃熸繛鎴炵矤濡插吋绻濆▓�?��灆缂佽埖鑹鹃～蹇撁洪鍕唶闁�?壈鎻徊鍝勎ｉ崼婵愭富闁靛洦澧庨崑鎾绘煕鐎ｎ偅灏电紒杈ㄦ尰閹峰懘�?��悧鍫熸瘞闂備胶鎳撻崵鏍�?��燂拷1闂傚倸鍊峰ù鍥敋瑜忛埀顒佺▓閺�?��鍒掑▎鎾崇婵＄偛鐨烽崑鎾�?礃閳轰胶绐為柣搴岛閺呮繄绮诲鑸�?拺閻犳亽鍔屽▍鎰版煙閸戙倖瀚� */
/* Below is the format of a complete received packet */
/* Header (a55a): 2 bytes */
/* Packet length: 1 byte */
/* ---Below is the payload, subject to change as per requirements--- */
/* hex_Yaw: 4 bytes (to be converted to IEEE 754 standard single precision floating point) */
/* hex_Pitch: 4 bytes (to be converted to IEEE 754 standard single precision floating point) */
/* ---Above is the payload--- */
/* Checksum: 1 byte */
/* Footer (ff): 1 byte */

// 闂傚倸鍊峰ù鍥敋瑜嶉～婵�?��閸岋妇绋忔繝銏ｅ煐閸旀牠宕曞Δ浣典簻闁哄倸鐏濋埛鏂库槈閹惧磭效闁哄矉缍佸�?�??鎼淬垹鏋戦梻浣哥－缁垰螞閸愵喖钃熺�?��儱鐗滃銊╂⒑閸涘﹥灏版慨妯稿�?閸掓帒鈻庨幇顒傜�?��佸綊鍋婇崢钘夆枍閵忋倖鈷戝ù鍏肩懅閸掓澘顭跨捄鐑樺枠鐎规洘鍨块�?��肩磼濡桨鐢婚梻浣虹帛椤ㄥ懘�?�繝鍥х闂侇剙绉甸埛鎴︽煕濠靛�?顏柨娑欐⒒缁辨挸�?奸崟顓犵崲濡ょ姷鍋涘Λ婵嗙暦濮椻偓椤㈡瑩鎳�?��濠冃у┑锛�?��閸婃牜鏁繝鍥ㄥ�?��柨鏇炲亞閺佸﹦鈧箍鍎遍ˇ浼村煕閹达附鐓曢柨鏃囶嚙�?炴銇勮箛鏇炴灈闁哄瞼鍠愰ˇ鐗堟償閳ュ啿绠ｉ梻浣告惈閺�?���?��鐐叉�?��瑰墽�?��弲鎼佹煥閻曞倹瀚�?
// define a complete packet
uint8_t packet[CV_PACKET_LENGTH];

/**
 * @brief          referee task
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
/**
 * @brief          闂傚倸鍊峰ù鍥х暦閻㈢ǹ绐楃€广儱鎷�?〒濠�?��閻戞ɑ鈷掗柣�?炴閵嗘帒顫濋敐鍛闁诲氦�??ú鏍礊婵犲洤绠栭柛鎾楀倹鍕�?��鑺ッˇ浠�?��濞嗘挻鈷掑ù锝�?��鐢埖銇�?��锝嗙鐎规洘鍔曡灃闁告侗鍘藉Σ顒勬⒑缂佹〒褰捤�?崱�?ヤ汗闁圭儤鎸告�?瑙勭箾閹�?��鍤柛銊︽緲閳绘捇顢橀悢铏�??濠电偛鐗嗛悘婵嗏枍濞嗘挻鐓欓柛鎴欏€�?��氾拷
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void referee_usart_task(void const *argument)
{

  memset(dma_buf, 0, sizeof(dma_buf)); // was 200: dma_buf is only 50 bytes
  memset(rx_data, 0, sizeof(rx_data));
  const fp32 *imu = get_INS_angle_point();
  while (1)
  {
    sprintf((char *)dma_buf, "A5%f,%f,%f", imu[0], imu[1], imu[2]);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)dma_buf, strlen((const char *)dma_buf));
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)rx_data, sizeof(rx_data));
    osDelay(1); // yield: without this the loop starved the OLED and idle tasks
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    // HAL_UART_Receive_DMA(&huart6, (uint8_t*)rx_rdata, 9);
    HAL_UART_Transmit_DMA(&huart6, (uint8_t *)dma_buf, strlen((const char *)dma_buf));
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    UART7_CommandRoute();
    HAL_UART_Receive_DMA(&huart6, (uint8_t *)rx_data, sizeof(rx_data));
  }
}

// use to find the first complete packet from rx_data, if find A5 and come with 5A, then
bool find_CpltPacket(void)
{
  // stop early enough that the whole packet fits inside rx_data (was reading past the end)
  for (size_t i = 0; i + sizeof(packet) <= sizeof(rx_data); i++)
  {
    // find A55A
    if (rx_data[i] == 0xA5 && rx_data[i + 1] == 0x5A)
    {
      // find FF
      if (rx_data[i + sizeof(packet) - 1] == 0xFF)
      {
        // copy the packet
        memcpy(packet, &rx_data[i], sizeof(packet));
        return true;
      }
    }
  }
  return false;
}

cv_Data_TypeDef cv_Data;
void UART7_CommandRoute(void)
{
  // get a complete packet by checking the rx_data
  bool packetFound = find_CpltPacket();

  // If no complete packet was found, return early and skip the rest
  if (!packetFound)
  {
    return;
  }

  HexToFloat yaw;
  HexToFloat pitch;

  // get Yaw
  uint32_t temp_Yaw = (uint32_t)packet[3] << 24 |
                      (uint32_t)packet[4] << 16 |
                      (uint32_t)packet[5] << 8 |
                      (uint32_t)packet[6];
  yaw.hex = temp_Yaw;
  cv_Data.yaw = yaw.floatValue;

  // get Pitch
  uint32_t temp_Pitch = (uint32_t)packet[7] << 24 |
                        (uint32_t)packet[8] << 16 |
                        (uint32_t)packet[9] << 8 |
                        (uint32_t)packet[10];
  pitch.hex = temp_Pitch;
  cv_Data.pitch = pitch.floatValue;

  // get Find_Target status
  cv_Data.find_target = (char)packet[12];

  // get the new cv data flag
  detect_hook(USER_USART_DATA_TOE);
  cv_Data.new_cv_data_flag = 1;
}

/**
 * @brief          single byte upacked
 * @param[in]      void
 * @retval         none
 */
/**
 * @brief          闂傚倸鍊搁崐椋庣矆娓氣偓楠炲鏁撻悩顔瑰亾閸愵喖骞㈡俊鐐存�?濡炶棄�?ｆ�?瀣垫晣闁绘�?娓瑰Σ鎰版煟閻斿摜�??俊�?㈠暣閻涱喗绻濋崶銊モ偓鐑芥煕濠靛棗�?柨娑欑矊閳规垿�?��鑲╁姶闂佸憡鎸诲銊╁极椤曗偓閸ㄦ儳鐣烽崶銊︻啎闂備浇娉曢崳锕傚�?閿燂�??
 * @param[in]      void
 * @retval         none
 */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while (fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    switch (p_obj->unpack_step)
    {
    case STEP_HEADER_SOF:
    {
      if (byte == sof)
      {
        p_obj->unpack_step = STEP_LENGTH_LOW;
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      else
      {
        p_obj->index = 0;
      }
    }
    break;

    case STEP_LENGTH_LOW:
    {
      p_obj->data_len = byte;
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_LENGTH_HIGH;
    }
    break;

    case STEP_LENGTH_HIGH:
    {
      p_obj->data_len |= (byte << 8);
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
      {
        p_obj->unpack_step = STEP_FRAME_SEQ;
      }
      else
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }
    }
    break;
    case STEP_FRAME_SEQ:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;
      p_obj->unpack_step = STEP_HEADER_CRC8;
    }
    break;

    case STEP_HEADER_CRC8:
    {
      p_obj->protocol_packet[p_obj->index++] = byte;

      if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
      {
        if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
        {
          p_obj->unpack_step = STEP_DATA_CRC16;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }
    }
    break;

    case STEP_DATA_CRC16:
    {
      if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
      }
      if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;

        if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          referee_data_solve(p_obj->protocol_packet);
        }
      }
    }
    break;

    default:
    {
      p_obj->unpack_step = STEP_HEADER_SOF;
      p_obj->index = 0;
    }
    break;
    }
  }
}

void USART6_IRQHandler(void)
{
  static volatile uint8_t res;
  if (USART6->SR & UART_FLAG_IDLE)
  {
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    static uint16_t this_time_rx_len = 0;

    if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[0], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
    else
    {
      __HAL_DMA_DISABLE(huart6.hdmarx);
      this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
      __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
      huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
      __HAL_DMA_ENABLE(huart6.hdmarx);
      fifo_s_puts(&referee_fifo, (char *)usart6_buf[1], this_time_rx_len);
      detect_hook(REFEREE_TOE);
    }
  }
}

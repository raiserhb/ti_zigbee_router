

import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;
import java.lang.reflect.UndeclaredThrowableException;
import java.math.BigInteger;
import java.security.GeneralSecurityException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

public class PYGen {

    /**
     * 转码位数 [1-8]
     */
    private static final int CODE_DIGITS = 4;

    /**
     * 时间位数
     */
    private static final int TIME_BLOCK_DIGITS = 4;

    /**
     * 初始化时间
     */
    private static final long INITIAL_TIME = 0;

    /**
     * 数子量级
     */
    private static final int[] DIGITS_POWER = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};




    public static void testPwd(){
        /** 接口传入当天8点和12点时间戳*/
        int startTime=12;
        int endTime=17;
        //String pwd=decryptPwd(startTime,endTime,"00158d00026c5415",12345678);
        String pwd= decryptPwd(startTime,endTime,"00158d00026c5369",12345678);
        encryptPwd(pwd);
    }


    /**
     * 生成离线密码0-24
     * @author eric
     * @date  11:08 AM 2018/12/17
     *
     * @Param: startHour 开始时间
     * @Param: endHour   结束时间
     * @Param: ieee      ieee地址
     * @Param: seed      密码种子
     * @return java.lang.String
     */
    public static String decryptPwd(Integer  startHour, Integer  endHour, String ieee, Integer seed){
        /**将ieee处理成大写*/
        String  ieeeUpperCase = ieee.toUpperCase();
        long  startTimestamp= getTimestamp(startHour);
        long  endTimestamp= getTimestamp(endHour);
        Long interval=endTimestamp-startTimestamp;
        System.out.println("算出开始时间和结束时间的间隔："+interval);
        /**通过KEY+SEED生成密码*/
        PYGen pyGen = new PYGen();
        String pwdBlock = pyGen.getPassword(ieeeUpperCase,seed,startTimestamp,interval);
        System.out.println("4位密码："+pwdBlock);
        /**生成时间块字符串，例如8点-12点：812*/
        String timeBlock=startHour+""+endHour;
        System.out.println("4位时间："+timeBlock);
        int  pwdBlockInt=Integer.parseInt(pwdBlock);
        int  timeBlockInt=Integer.parseInt(timeBlock);
        int sub=pwdBlockInt-timeBlockInt;
        int subAbs=Math.abs(sub);
        System.out.println("4位密码和4位时间差的绝对值："+subAbs);

        /**补齐时间位数*/
        String subAbsStr=fillGap(""+subAbs,TIME_BLOCK_DIGITS);
        System.out.println("差的绝对值少于4位数补0："+subAbsStr);
        String pwd=pwdBlock +subAbsStr;
        System.out.println("离线密码:"+pwd+" 开始:"+startTimestamp+" 结束:"+endTimestamp+" ieee:"+ieeeUpperCase+" seed:"+seed +" 有效期:"+startHour+"~"+endHour);
        return  pwd;
    }



    /**
     * 解析密码
     * @author eric
     * @date  7:41 PM 2019/2/26
     *
     * @Param: pwd
     * @return void
     */
    public static void encryptPwd(String  pwd){

        System.out.println("----------解析出密码-----------------");
        String pwdBlock=pwd.substring(0,CODE_DIGITS);
        String subBlock=pwd.substring(CODE_DIGITS,pwd.length());
        System.out.println("4位密码："+pwdBlock);
        System.out.println("4位差值："+subBlock);
        int pwdBlockInt=Integer.parseInt(pwdBlock);
        int subBlockInt=Integer.parseInt(subBlock);

        String timeBlock1=String.valueOf(pwdBlockInt-subBlockInt);
        String timeBlock2=String.valueOf(pwdBlockInt+subBlockInt);
        System.out.println("---情况1---");
        encryptTime(timeBlock1);
        System.out.println("---情况2---");
        encryptTime(timeBlock2);
    }

    public static void encryptTime(String  timeBlock){
        if(timeBlock.length()>TIME_BLOCK_DIGITS){
            System.out.println("时间位数超出"+TIME_BLOCK_DIGITS+"不满足条件");
        }else{
            if(timeBlock.length()<TIME_BLOCK_DIGITS){
                timeBlock=fillGap(timeBlock,TIME_BLOCK_DIGITS);
                System.out.println("时间位数小于"+TIME_BLOCK_DIGITS+"补位："+timeBlock);
            }
            String  startHour=timeBlock.substring(0,2);
            String  endHour=timeBlock.substring(2,timeBlock.length());
            System.out.println("开始小时："+startHour);
            System.out.println("结束小时："+endHour);
        }
    }



    /**
     * @author eric
     * @date  11:09 AM 2018/12/17
     *
     * @Param: ieee  ieee地址
     * @Param: seed  密码种子
     * @Param: interval 开始和结束时间间隔
     * @return java.lang.String
     */
    String getPassword(String ieee,Integer seed,long startTime,Long interval) {
        String strSeed = Integer.toHexString(seed).toUpperCase();
        StringBuilder seedBuilder = new StringBuilder(strSeed);
        while (seedBuilder.length() < 8) {
            seedBuilder.insert(0, "0");
        }
        strSeed = seedBuilder.toString();
        System.out.println("seed :" + Integer.toString(seed) + ", HexStr is :" + strSeed);
        String totp = generateMyTOTP(ieee,strSeed.toUpperCase(), startTime, interval);
        return totp;

    }

    /**
     * 生成一次性密码
     *
     * @param seed     ---> "",key
     * @param now      开始时间
     * @param interval 周期 ---> seed 密码
     * @return String
     */
    public String generateMyTOTP(String ieee,String seed, Long now, Long interval) {
        if (now == null || now == 0) {
            now = new Date().getTime();
        }
        String time = Long.toHexString(timeFactor(now, interval)).toUpperCase();
        return generateTOTP(seed + ieee, time); //
    }
    private static String generateTOTP(String key, String time) {
        return generateTOTP(key, time, "HmacSHA1"); // modify by au 2018/11/19
    }

    private static String generateTOTP(String key, String time, String crypto) {
        StringBuilder timeBuilder = new StringBuilder(time);
        while (timeBuilder.length() < 8) // modify by au 2018/11/19
            timeBuilder.insert(0, "0");
        time = timeBuilder.toString();
        byte[] msg = hexStr2Bytes(time); // modify by au 2018/11/19
        byte[] k = hexStr2Bytes(key); // modify by au 2018/11/19
        byte[] hash = hmac_sha(crypto, k, msg);
        return truncate(hash);
    }
    /**
     * 获取动态因子
     *
     * @param targetTime 指定时间
     * @return long
     */
    private static long timeFactor(long targetTime, Long interval) {
        long y = (targetTime - INITIAL_TIME) / 1000 / (interval / 1000);
        return (targetTime - INITIAL_TIME) / 1000 / (interval / 1000);
    }

    /**
     * 哈希加密
     *
     * @param crypto   加密算法
     * @param keyBytes 密钥数组
     * @param text     加密内容
     * @return byte[]
     */
    private static byte[] hmac_sha(String crypto, byte[] keyBytes, byte[] text) {
        try {
            Mac hmac;
            hmac = Mac.getInstance(crypto);
            SecretKeySpec macKey = new SecretKeySpec(keyBytes, "AES");
            hmac.init(macKey);
            return hmac.doFinal(text);
        } catch (GeneralSecurityException gse) {
            throw new UndeclaredThrowableException(gse);
        }
    }

    private static byte[] hexStr2Bytes(String hex) {
        byte[] bArray = new BigInteger("10" + hex, 16).toByteArray();
        byte[] ret = new byte[bArray.length - 1];
        System.arraycopy(bArray, 1, ret, 0, ret.length);
        return ret;
    }

    private static String bytesToHexString(byte[] src) {
        StringBuilder stringBuilder = new StringBuilder("");
        if (src == null || src.length <= 0) {
            return null;
        }
        for (int i = 0; i < src.length; i++) {
            int v = src[i] & 0xFF;
            String hv = Integer.toHexString(v).toUpperCase();
            if (hv.length() < 2) {
                stringBuilder.append(0);
            }
            stringBuilder.append(hv);
        }
        return stringBuilder.toString();
    }

    /**
     * 补位
     * @author eric
     * @date  11:12 AM 2018/12/17
     *
     * @Param: str 密码
     * @Param: digits 达到位数
     * @return java.lang.String
     */
    public static String fillGap(String str,int digits){
        int a=digits-str.length();
        String zero="0";
        for (int i = 0; i <a ; i++) {
            str=zero+str;
        }
        return str;
    }


    /**
     * 截断函数
     *
     * @param target 20字节的字符串
     * @return String
     */
    private static String truncate(byte[] target) {
        StringBuilder result;
        int offset = 0; // modify by au 2018/11/19
        int binary = ((target[offset] & 0x7f) << 24) | ((target[offset + 1] & 0xff) << 16) | ((target[offset + 2] & 0xff) << 8) | (target[offset + 3] & 0xff);

        int otp = binary % DIGITS_POWER[CODE_DIGITS];
        System.out.println("otp is " + Integer.toString(otp));
        result = new StringBuilder(Integer.toString(otp));
        while (result.length() < CODE_DIGITS) {
            result.insert(0, "0");
        }
        return result.toString();
    }


    /**获取当天某一时刻时间戳*/
    public static long getTimestamp(int hour){
        Calendar cal = Calendar.getInstance();
        cal.set(Calendar.HOUR_OF_DAY, hour);
        cal.set(Calendar.SECOND, 0);
        cal.set(Calendar.MINUTE, 0);
        cal.set(Calendar.MILLISECOND, 0);
        return cal.getTimeInMillis();
    }


     public static  Integer getSeed(){
         int seed = new Random().nextInt(99999999);
         if (seed < 10000000) {
             seed += 10000000;
         }
         System.out.println("种子:"+seed);
         return seed;
     }

    public static void main(String[] args) {
        // getSeed();
        testPwd();

        //getTimestamp(24);
        System.out.println( getTimestamp(0));
        System.out.println( getTimestamp(24));

    }




}

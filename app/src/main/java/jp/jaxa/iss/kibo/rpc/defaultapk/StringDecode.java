package jp.jaxa.iss.kibo.rpc.defaultapk;

public class StringDecode {
    public static int p = 0;
    public static float x = 0;
    public static float y = 0;
    public static float z = 0;
    public static String tempStr = null;

    public void setString(String newString) {
        tempStr = newString;
        split(newString);

    }
    private void split(String string) {
        String cut = string.replaceAll("[(){}]", "");
        System.out.println(cut);
        String[] textSplit = cut.split(",");

        p = Integer.parseInt(textSplit[0].substring(4));
        x = Float.parseFloat(textSplit[1].substring(4));
        y = Float.parseFloat(textSplit[2].substring(4));
        z = Float.parseFloat(textSplit[3].substring(4));

    }
}

package jp.jaxa.iss.kibo.rpc.defaultapk;

public class StringDecode {
    private static int p = 0;
    private static float x = 0;
    private static float y = 0;
    private static float z = 0;
    private static String tempStr = null;

    public void setString(String newString) {
        tempStr = newString;
        split(newString);

    }
    public float getPosX() {
        return x;
    }

    public float getPosY() {
        return y;
    }

    public float getPosZ() {
        return z;
    }

    public int getPattern() {
        return p;
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

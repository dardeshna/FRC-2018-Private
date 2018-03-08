package viewer;

/**
 * Data class that contains information about how to run the viewer.
 * This is serialized via JSON to a file at the path {@link ViewerMain#RELATIVE_CONFIG_PATH}.
 */
public class Config {

    private static final boolean DEFAULT_DEBUGGING = false;
    private static final int DEFAULT_PORT = 1180;
    private static final String DEFAULT_IP = "localhost";
    private static final double DEFAULT_PREVIEW_SCALE = 2.25d;

    public final String address;
    public final int port;
    public final double previewScale;
    public final boolean debugMode;

    /**
     * Default config, used if no config file is found or is broken.
     */
    public Config() {
        address = DEFAULT_IP;
        port = DEFAULT_PORT;
        previewScale = DEFAULT_PREVIEW_SCALE;
        debugMode = DEFAULT_DEBUGGING;
    }
}

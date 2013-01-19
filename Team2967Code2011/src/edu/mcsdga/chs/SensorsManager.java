package edu.mcsdga.chs;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStationLCD;
/**
  SensorsManager is a class for working with FRC robot sensors.
  @author David A. Rush
 */
public class SensorsManager implements Runnable {

  /**
    getInstance() is the one and only proper way to get an instance of a SensorManager object.
    In fact, there should only ever be one instance of a SensorManager object.
    All calls to getInstance() will return the same instance.
   */
  public static synchronized SensorsManager getInstance() {
    // This method is synchronized to prevent two different but nearly simultaneous invocations of getInstance()
    // from each causing the creation of a new SensorManager object.
    // See "Head First Design Patterns", page 180.
    if (sm==null) {
      sm = new SensorsManager();
    }
    return sm;
  }

  /* test */

  /*
  public static SensorsManager getInstance(boolean wait_until_ready) {
    getInstance();
    if (wait_until_ready) {
      sm.waitUntilReady();
    }
    return sm;
  }
   */

  public void waitUntilReady() {
    while (!ready_flag) {
      try {
        Thread.sleep(10);
      } catch (InterruptedException ex) {
        ex.printStackTrace();
      }
    }
  }

  public void start() {
    thread = new Thread(this);
    thread.start();
  }

  public void stop() {
    quit_flag=true;
  }

  public void startCountdown(long minutes, long seconds) {
    countdown_end = System.currentTimeMillis()+(minutes*60+seconds)*1000L;
  }

  public void setDriverStationScreen(DriverStationLCD ds) {
      this.driver_station_screen = ds;
  }

  public void setJoystickPorts(int[] joystick_ports) {
    this.joystick_count = joystick_ports.length;
    this.joystick_ports = new int[joystick_count];
    this.joysticks = new Joystick[joystick_count];
    this.joystick_buttons = new boolean[joystick_count][JOYSTICK_BUTTON_COUNT];
    for(int j=0;j<joystick_count;j++) {
      joysticks[j] = new Joystick(joystick_ports[j]);
      // Do any other common joystick configuration here.
      // Iterate over the buttons and set 'em false.
      for(int b=1;b<=JOYSTICK_BUTTON_COUNT;b++) {
        joystick_buttons[j][b] = false;
      }
    }
  }

  public String getJoystickString() {
    String tmp = "Joysticks:";
    for(int j=0;j<joystick_count;j++) {
        if (j!=0) {
            tmp += ", ";
        } else {
            tmp += " ";
        }
        tmp += " joystick[j]: " + joysticks[j];
    }
    return tmp;
  }


  public void setGyroSlot(int s) {
      gyro_slot = s;
  }
  public void setGyroChannel(int c) {
      gyro_channel = c;
  }

  public void setPhotoSensors(int[] sensor_channel_nums) {
    photo_sensor_count = sensor_channel_nums.length;
    this.photo_sensor_channels = new int[photo_sensor_count];
    photo_sensors = new AnalogTrigger[photo_sensor_count];
    for(int j=0;j<photo_sensor_count;j++) {
      this.photo_sensor_channels[j] = sensor_channel_nums[j];
        photo_sensors[j] = new AnalogTrigger(photo_sensor_channels[j]);
        photo_sensors[j].setLimitsVoltage(0.5,0.5);
    }

  }

  public String getPhotoSensorsString() {
    if (photo_sensor_count==0) {
      throw new RuntimeException("Photo sensors not initialized.");
    }
    String tmp = "photo sensors: ";
    for(int j=0;j<photo_sensor_count;j++) {
      if (j>0) {
        tmp += ", ";
      } else {
        tmp += "  ";
      }
      tmp += j+" (ch "+photo_sensor_channels[j]+"] = "+(photo_sensors[j].getTriggerState()?"true ":"false");
    }
    return tmp;
  }

  /**
   * Gets the heading that the gyro is currently facing.
   * @return the current heading
   */
  /*
  public double getGyroAngle() {
    // need to implement something useful here
    if (gyro==null) {
      throw new RuntimeException("Gyro is not initialized.");
    }
    return gyro_angle;
  }
   */

  private int photo_sensor_count = 0;
  private int[] photo_sensor_channels = null;

  private int joystick_count = 0;
  private int[] joystick_ports = null;
  private Joystick[] joysticks;
  private static final int JOYSTICK_BUTTON_COUNT=16;
  private boolean[][] joystick_buttons;

  private Thread thread = null;
  private boolean quit_flag = false; // set this to true when we want run() to exit
  private boolean ready_flag = false; // Once fully initialized and read, this will be true.
  private long run_loop_counter = 0;
  private static final long max_loops = -1; // maximum number of loops to run before quitting
  // max_loops < 0 means loop forever
  private boolean running = false;

  private Gyro gyro = null;
  private int gyro_slot = 0;
  private int gyro_channel = -1;
  private double gyro_angle = Double.NaN;
  private long start_time = 0L; // for keeping track of the heart beat
  private long countdown_end = 0L;  // when the countdown will hit zero
  private boolean heart_beat_tic = false; // tic (vs. tock) controls whether H or B is displayed

  private AnalogTrigger[] photo_sensors = null;
  private DriverStationLCD driver_station_screen = null;

  /**
    The constructor is private, preventing other classes from calling in directly.
    The proper way to get a SensorManager object is to call SensorManager.getInstance().
   */
  private static SensorsManager sm; // One, and only one instance called "sm".

  /**
    The private constructor.
   */
  private SensorsManager() {
    System.out.println("Creating the one and only SensorManager instance.");
    //Thread t = new Thread(this);
    //t.start();
    start_time = System.currentTimeMillis();
    countdown_end = 0L;
  }

  /**
    Since this class implements Runnable, it must have a "run()" method.
   */
  public void run() {
    running = true;
    // do initializations
    if (gyro_channel>=0) {
      Gyro gyro = new Gyro(gyro_channel);
    }
    ready_flag = true;
    // Now run the main loop
    boolean do_update = false;
    while (!quit_flag) {
      if (max_loops>=0 && run_loop_counter>=max_loops) {
        quit_flag=true;
      } else {
        do_update = false;
        long now = System.currentTimeMillis();
        boolean old_heart_beat_tic = heart_beat_tic;
        if (now%1000<500) {
          heart_beat_tic = true;
        } else {
          heart_beat_tic = false;
        }
        if (heart_beat_tic != old_heart_beat_tic) {
            do_update = true;
        }
        // gyro_angle = gyro.getAngle();
        /*
        try {
          Thread.sleep(10);
          // System.out.println("run_loop_counter = "+run_loop_counter);
        } catch (InterruptedException ex) {
          ex.printStackTrace();
        }
         */
        if (do_update) {
          long countdown_remaining = countdown_end - System.currentTimeMillis();
          if (countdown_remaining<0) {
            countdown_remaining = 0L;
          }
          update(System.currentTimeMillis() - start_time, countdown_remaining);
        }
        run_loop_counter++;
      }
      // Thread.yield(); // polite, but not strictly necessary
      try { Thread.sleep(50); } catch (InterruptedException ex) {}
    }
    running = false;
  }

  private void update(long lifetime, long countdown) {
      StringBuffer sb = new StringBuffer();
      if (heart_beat_tic) {
        sb.append("H ");
      } else {
        sb.append(" B");
      }
      if (driver_station_screen!=null) {
        driver_station_screen.println(DriverStationLCD.Line.kUser6,1,sb);
        // Build a time string for display
        String time = formatTime(lifetime);
        driver_station_screen.println(DriverStationLCD.Line.kUser6,4,time);
        time = formatTime(lifetime);
        // 123456789012345678921234
        // HB 0000:00:00 0000:00:00
        driver_station_screen.println(DriverStationLCD.Line.kUser6,15,time);
        driver_station_screen.updateLCD();
      }
  }


  private String formatTime(long now) {
    long seconds = now/1000;
    long minutes = seconds/60;
    seconds -= minutes*60;
    long hours = minutes/60;
    minutes -= hours*60;
    StringBuffer sb = new StringBuffer();
    // sb.append(df_2dig.format(hours));
    sb.append(hours);
    sb.append(":");
    appendTwoDigitNumber(minutes,sb);
    sb.append(":");
    appendTwoDigitNumber(seconds,sb);
    return sb.toString();
  }

  private void appendTwoDigitNumber(long n,StringBuffer sb) {
    if (n<10) {
      sb.append('0');
    }
    sb.append(n);
  }


  /*
  public static void main(String[] args) {
    // Self Test!
    SensorsManager my_sm = SensorsManager.getInstance(true);
    System.out.println("gyro heading = "+my_sm.getGyroAngle());
    System.out.println("main is done.  Goodbye.");
  }
   */

}
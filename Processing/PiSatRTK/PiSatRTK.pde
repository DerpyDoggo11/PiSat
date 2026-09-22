// ============================================================================
//  PiSat RTK  --  rover telemetry & path visualiser  (Processing 4.x)
//
//  Connects over TCP to the rover ESP32's telemetry server and draws the rover
//  driving its GPS waypoint path over a SATELLITE background (Esri World Imagery,
//  free/no key), with a live HUD: fix quality, position, satellites, HDOP,
//  speed/course, cross-track heading error, correction age and per-wheel PWM.
//  Press [s] to toggle the satellite view.
//
//  SETUP
//    1. Flash the firmware and power up base + rover (see RTK_SETUP_INSTRUCTIONS.txt).
//    2. Join the WiFi network "PiSat-RTK" (password pisat-rtk-1234) on this computer.
//       NOTE: satellite imagery needs INTERNET, which the "PiSat-RTK" AP does not
//       provide. For imagery, run on a machine that can reach the internet AND the
//       rover (e.g. a second WiFi/Ethernet), or the map area will just not load
//       (telemetry/path still work fine over the AP).
//    3. Set ROVER_HOST below to the rover's IP (192.168.4.1 by default) and run.
//
//  PROTOCOL (newline-terminated ASCII lines from the rover)
//    M,<maxPwm>,<wpRadius>,<wpCount>,<boundaryCount>
//    W,<i>,<lat>,<lon>              waypoint i
//    B,<i>,<lat>,<lon>              boundary vertex i
//    O,<lat>,<lon>                  local-frame origin (first fix)
//    T,<ms>,<fixQual>,<lat>,<lon>,<course>,<speed>,<sats>,<hdop>,
//      <seg>,<tgtBearing>,<hdgErr>,<done>,<rtcmAgeMs>,<pwmR>,<pwmL>,<spdHzR>,<spdHzL>
// ============================================================================

import processing.net.*;

// ---- connection ----
// SoftAP mode: rover is 192.168.4.1. NTRIP mode: rover is a station on your
// WiFi -- set this to its IP (printed on the C3 serial) or "pisat-rover.local".
String ROVER_HOST = "192.168.4.1";
int    ROVER_PORT = 8888;   // telemetry (read)
int    CMD_PORT   = 8889;   // commands (path + WASD, write)
Client rover;               // telemetry client
Client cmd;                 // command client
boolean connected = false;
int    lastRxMs   = 0;
int    lastConnectAttempt = 0;
int    badLines   = 0;   // corrupted telemetry lines skipped (link-health hint)
boolean cmdWasUp  = false;  // cmd-link edge detect (re-push path on reconnect)

// ---- manual WASD control ----
boolean kW=false, kA=false, kS=false, kD=false;
int     lastManMs = 0;
boolean wasDriving = false;
int     driveMode = 0;      // from telemetry: 0 auto, 1 manual, 2 hold

// ---- offline project (from the PiSat planner) ----
boolean offlineProj = false;
String  SHARED = "../shared/";

// ---- world model ----
boolean originSet = false;
double  originLat = 0, originLon = 0;
double  mPerDegLat = 111320.0, mPerDegLon = 111320.0;

ArrayList<double[]> waypoints = new ArrayList<double[]>();  // {lat,lon}
ArrayList<double[]> boundary  = new ArrayList<double[]>();
double baseLat = 0, baseLon = 0; boolean haveBase = false;  // base station GPS dot
float  offE = 0, offN = 0;   // manual GPS offset (metres E/N) to align the fix
float  vbat = 0;             // PiSat battery V (0 = no batt / USB, ignore)
float  PPR  = 45;            // wheel hall pulses per rev (MowerBot value) -> RPM
float   wpRadius = 1.5;
float   maxPwm   = 40;

// ---- live telemetry ----
int     fixQual = 0;
double  lat = 0, lon = 0;
float   course = 0, speed = 0, hdop = 99.9;
int     sats = 0, seg = 0;
float   tgtBearing = 0, hdgErr = 0;
boolean done = false;
long    rtcmAge = 999999;
float   pwmR = 0, pwmL = 0;
float   spdHzR = 0, spdHzL = 0;   // wheel-speed feedback (pulses/sec)
long    fixMs = 0;

ArrayList<float[]> trail = new ArrayList<float[]>();  // rover EN history (metres)

PFont mono;

void settings() { size(1180, 760); }

void setup() {
  surface.setTitle("PiSat RTK  --  rover path & telemetry");
  mono = createFont("Consolas", 13);
  textFont(mono);
  loadProject();     // offline path/boundary + cached imagery from the planner
  connect();
}

void connect() {
  lastConnectAttempt = millis();
  try {
    rover = new Client(this, ROVER_HOST, ROVER_PORT);
    connected = rover.active();
  } catch (Exception e) { connected = false; }
  try { cmd = new Client(this, ROVER_HOST, CMD_PORT); cmdWasUp = false; }
  catch (Exception e) { }
}

// Load the planner's cached project: path + boundary + satellite image (offline).
void loadProject() {
  String[] lines = loadStrings(sketchPath(SHARED + "project.pisat"));
  if (lines == null) return;                 // no project -> stay online
  waypoints.clear(); boundary.clear();
  for (String ln : lines) {
    String[] t = splitTokens(trim(ln), " ");
    if (t.length == 0) continue;
    if (t[0].equals("base") && t.length>=3) { baseLat=Double.parseDouble(t[1]); baseLon=Double.parseDouble(t[2]); haveBase=true; }
    else if (t[0].equals("wp")  && t.length>=3) waypoints.add(new double[]{Double.parseDouble(t[1]), Double.parseDouble(t[2])});
    else if (t[0].equals("bnd") && t.length>=3) boundary.add(new double[]{Double.parseDouble(t[1]), Double.parseDouble(t[2])});
    else if (t[0].equals("bounds") && t.length>=5) {
      satW=Double.parseDouble(t[1]); satS=Double.parseDouble(t[2]);
      satE=Double.parseDouble(t[3]); satN=Double.parseDouble(t[4]);
    }
  }
  PImage img = loadImage(sketchPath(SHARED + "project.jpg"));
  if (img != null && img.width > 0) { sat = img; offlineProj = true; satOn = true; }
}

// Send a checksummed command line to the rover ("<body>*HH\n").
void sendCmd(String body) {
  if (cmd == null || !cmd.active()) return;
  int cs = 0; for (int i=1; i<body.length(); i++) cs ^= body.charAt(i);  // XOR after '$' (matches firmware)
  cmd.write(body + "*" + hex(cs & 0xFF, 2) + "\n");
}

// Push the offline project's path + boundary to the rover once, on connect.
void sendPath() {
  int n = waypoints.size();
  for (int i=0;i<n;i++) sendCmd("$WPT," + i + "," + n + "," + nfd(waypoints.get(i)[0],7) + "," + nfd(waypoints.get(i)[1],7));
  int m = boundary.size();
  for (int i=0;i<m;i++) sendCmd("$BND," + i + "," + m + "," + nfd(boundary.get(i)[0],7) + "," + nfd(boundary.get(i)[1],7));
}

// East/North metres from lat/lon in the rover's local frame.
double[] toLocal(double la, double lo) {
  return new double[] { (lo - originLon) * mPerDegLon, (la - originLat) * mPerDegLat };
}

void draw() {
  background(17, 20, 26);
  pumpNetwork();

  // push the planned path to the rover once its command link is up
  boolean cmdUp = cmd != null && cmd.active();
  if (cmdUp && !cmdWasUp) {          // link just came up (or reconnected)
    if (offlineProj) sendPath();     // (re)push path+boundary -> rover RAM
    setMode(1);                      // start in MANUAL for testing
  }
  cmdWasUp = cmdUp;
  serviceManual();

  if (originSet) { drawField(); }
  drawHud();
  drawManualPad();

  // auto-reconnect
  if ((rover == null || !rover.active()) && millis() - lastConnectAttempt > 2000) {
    connected = false;
    connect();
  }
}

// Effective button states = held key OR mouse pressed over the on-screen button.
boolean eW=false, eA=false, eS=false, eD=false;

float[] keyPos(char k) {
  float x0 = 20, top = height - 176;
  if (k=='W') return new float[]{x0+62, top+46};
  if (k=='A') return new float[]{x0+30, top+80};
  if (k=='S') return new float[]{x0+62, top+80};
  return new float[]{x0+94, top+80};            // 'D'
}
boolean overKey(char k) { float[] p = keyPos(k); return dist(mouseX,mouseY,p[0],p[1]) < 18; }

// Send WASD drive at ~10 Hz while any button is active; one stop on release.
void serviceManual() {
  boolean md = mousePressed && (mouseButton == LEFT);
  eW = kW || (md && overKey('W'));
  eA = kA || (md && overKey('A'));
  eS = kS || (md && overKey('S'));
  eD = kD || (md && overKey('D'));

  boolean driving = eW || eA || eS || eD;
  if (millis() - lastManMs < 100) return;
  lastManMs = millis();
  if (driving) {
    float throttle = (eW?1:0) - (eS?1:0);
    float turn     = (eD?1:0) - (eA?1:0);
    sendCmd("$MAN," + nf(throttle,0,2) + "," + nf(turn,0,2));
    wasDriving = true;
  } else if (wasDriving) {
    sendCmd("$MAN,0.00,0.00");            // one explicit stop
    wasDriving = false;
  }
}

void keyPressed() {
  if      (key=='w'||key=='W') kW=true;
  else if (key=='a'||key=='A') kA=true;
  else if (key=='s'||key=='S') kS=true;
  else if (key=='d'||key=='D') kD=true;
  else if (key=='g'||key=='G') sendCmd("$AUTO");          // resume autonomous path
  else if (key==' ')           sendCmd("$HOLD");          // toggle brake hold
  else if (key=='m'||key=='M') {                          // toggle map background
    satOn = !satOn;
    if (satOn && !offlineProj) { sat = null; satFailed = false; lastSatReq = 0; }
  }
  else if (key=='o'||key=='O') { offE = 0; offN = 0; }    // reset GPS offset
  else if (key=='[') { maxPwm = max(0,  maxPwm-5); sendCmd("$SPD," + int(maxPwm)); }  // slower
  else if (key==']') { maxPwm = min(255,maxPwm+5); sendCmd("$SPD," + int(maxPwm)); }  // faster
  else if (key=='p'||key=='P') sendPath();   // re-push path/boundary to rover
  else if (key==CODED) {                                  // arrows nudge GPS offset
    float step = 0.2;                                     // metres per press
    if      (keyCode==UP)    offN += step;
    else if (keyCode==DOWN)  offN -= step;
    else if (keyCode==RIGHT) offE += step;
    else if (keyCode==LEFT)  offE -= step;
  }
}
void keyReleased() {
  if      (key=='w'||key=='W') kW=false;
  else if (key=='a'||key=='A') kA=false;
  else if (key=='s'||key=='S') kS=false;
  else if (key=='d'||key=='D') kD=false;
}

// ----------------------------------------------------------------------------
//  NETWORK
// ----------------------------------------------------------------------------
void pumpNetwork() {
  if (rover == null || !rover.active()) return;
  while (rover.available() > 0) {
    String line = rover.readStringUntil('\n');
    if (line == null) break;
    line = trim(line);
    if (line.length() > 0) { handleLine(line); lastRxMs = millis(); connected = true; }
  }
}

// Java's Double/Long parsers throw on junk; a corrupted telemetry line must be
// skipped, never crash the sketch. Parse into locals, then commit on success.
double pd(String s) { return Double.parseDouble(trim(s)); }
long   pl(String s) { return Long.parseLong(trim(s)); }
float  pf(String s) { return Float.parseFloat(trim(s)); }
int    pi(String s) { return Integer.parseInt(trim(s)); }

void handleLine(String line) {
  try {
    // Verify the "<body>*HH" XOR checksum; reject a corrupted line outright.
    int star = line.indexOf('*');
    if (star >= 0) {
      int cs = 0;
      for (int i = 0; i < star; i++) cs ^= line.charAt(i);
      if (cs != Integer.parseInt(trim(line.substring(star + 1)), 16)) { badLines++; return; }
      line = line.substring(0, star);
    }

    String[] f = split(line, ',');
    if (f == null || f.length == 0 || f[0] == null) return;

    if (f[0].equals("T") && f.length >= 16) {
      int     _fixQual = pi(f[2]);
      double  _lat = pd(f[3]),  _lon = pd(f[4]);
      float   _course = pf(f[5]), _speed = pf(f[6]);
      int     _sats = pi(f[7]);
      float   _hdop = pf(f[8]);
      int     _seg = pi(f[9]);
      float   _tgt = pf(f[10]), _err = pf(f[11]);
      boolean _done = pi(f[12]) != 0;
      long    _age = pl(f[13]);
      float   _pwmR = pf(f[14]), _pwmL = pf(f[15]);
      float   _sR = spdHzR, _sL = spdHzL;
      if (f.length >= 18) { _sR = pf(f[16]); _sL = pf(f[17]); }
      if (f.length >= 19) driveMode = pi(f[18]);   // 0 auto, 1 manual, 2 hold
      if (f.length >= 21) { vbat = pf(f[19]); maxPwm = pf(f[20]); }

      // all fields parsed OK -> commit
      fixQual=_fixQual; lat=_lat; lon=_lon; course=_course; speed=_speed;
      sats=_sats; hdop=_hdop; seg=_seg; tgtBearing=_tgt; hdgErr=_err;
      done=_done; rtcmAge=_age; pwmR=_pwmR; pwmL=_pwmL; spdHzR=_sR; spdHzL=_sL;
      fixMs = millis();

      if (fixQual >= 1 && originSet) {
        double[] en = toLocal(lat, lon);
        float[] p = new float[] { (float)en[0], (float)en[1] };
        if (trail.isEmpty() || dist(p[0], p[1], trail.get(trail.size()-1)[0], trail.get(trail.size()-1)[1]) > 0.05)
          trail.add(p);
        if (trail.size() > 4000) trail.remove(0);
      }
    } else if (f[0].equals("BP") && f.length >= 3) {
      baseLat = pd(f[1]); baseLon = pd(f[2]); haveBase = true;
    } else if (f[0].equals("O") && f.length >= 3) {
      setOrigin(pd(f[1]), pd(f[2]));
    } else if (f[0].equals("W") && f.length >= 4) {
      int i = pi(f[1]); if (i < 0 || i > 4096) return;
      ensureSize(waypoints, i);
      waypoints.set(i, new double[]{ pd(f[2]), pd(f[3]) });
    } else if (f[0].equals("B") && f.length >= 4) {
      int i = pi(f[1]); if (i < 0 || i > 4096) return;
      ensureSize(boundary, i);
      boundary.set(i, new double[]{ pd(f[2]), pd(f[3]) });
    } else if (f[0].equals("M") && f.length >= 3) {
      maxPwm = pf(f[1]); wpRadius = pf(f[2]);
    }
  } catch (Exception e) {
    badLines++;          // corrupt line -> skip it, keep the last good values
  }
}

void ensureSize(ArrayList<double[]> a, int i) {
  while (a.size() <= i) a.add(new double[]{0, 0});
}

void setOrigin(double la, double lo) {
  originLat = la; originLon = lo;
  mPerDegLat = 111320.0;
  mPerDegLon = 111320.0 * Math.cos(la * Math.PI / 180.0);
  originSet = true;
}

// ----------------------------------------------------------------------------
//  VIEW TRANSFORM (auto-fit everything we know about)
// ----------------------------------------------------------------------------
float viewScale = 20;   // px per metre
float viewCE = 0, viewCN = 0;

// ---- satellite background (Esri World Imagery -- free, no API key) ----
boolean satOn = true;
PImage  sat = null;                    // currently displayed tile
double  satW=0, satE=0, satS=0, satN=0;// its geo-bounds (lon W/E, lat S/N)
PImage  satPending = null;             // async request in flight
double  pW=0, pE=0, pS=0, pN=0;        // pending request's bounds
int     lastSatReq = 0;
boolean satFailed = false;             // last fetch failed (usually = no internet)

double lonAt(double e) { return originLon + e / mPerDegLon; }  // metres E -> lon
double latAt(double n) { return originLat + n / mPerDegLat; }  // metres N -> lat

void computeView() {
  float minE = 1e9, maxE = -1e9, minN = 1e9, maxN = -1e9;
  boolean any = false;
  for (double[] w : waypoints) { double[] en = toLocal(w[0], w[1]); any=true;
    minE=min(minE,(float)en[0]); maxE=max(maxE,(float)en[0]); minN=min(minN,(float)en[1]); maxN=max(maxN,(float)en[1]); }
  for (double[] b : boundary)  { double[] en = toLocal(b[0], b[1]); any=true;
    minE=min(minE,(float)en[0]); maxE=max(maxE,(float)en[0]); minN=min(minN,(float)en[1]); maxN=max(maxN,(float)en[1]); }
  if (fixQual >= 1) { double[] en = toLocal(lat, lon); any=true;
    minE=min(minE,(float)en[0]); maxE=max(maxE,(float)en[0]); minN=min(minN,(float)en[1]); maxN=max(maxN,(float)en[1]); }
  if (!any) { minE=-10; maxE=10; minN=-10; maxN=10; }

  float padM = 3;
  minE-=padM; maxE+=padM; minN-=padM; maxN+=padM;
  viewCE = (minE+maxE)/2; viewCN = (minN+maxN)/2;
  float spanE = max(maxE-minE, 1), spanN = max(maxN-minN, 1);
  float fieldW = width - 340, fieldH = height - 40;
  viewScale = constrain(min(fieldW/spanE, fieldH/spanN), 0.5, 60);
}

float sx(float e) { return 20 + (width-340)/2.0 + (e - viewCE) * viewScale; }
float sy(float n) { return 20 + (height-40)/2.0 - (n - viewCN) * viewScale; }

// Fetch a fresh satellite tile when the view moves outside the loaded one.
// Esri World Imagery XYZ tiles (reliable; the export op 500s). Fetch+stitch.
int  lon2tx(double lon, int z) { return (int)Math.floor((lon+180.0)/360.0*(1L<<z)); }
int  lat2ty(double lat, int z) { double r=Math.toRadians(lat);
  return (int)Math.floor((1-Math.log(Math.tan(r)+1/Math.cos(r))/Math.PI)/2*(1L<<z)); }
double tx2lon(int x, int z) { return x/(double)(1L<<z)*360.0-180.0; }
double ty2lat(int y, int z) { double m=Math.PI-2.0*Math.PI*y/(1L<<z); return Math.toDegrees(Math.atan(Math.sinh(m))); }
int chooseZoom(double w, double s, double e, double n) {
  for (int z=19; z>=15; z--) { int nx=lon2tx(e,z)-lon2tx(w,z)+1, ny=lat2ty(s,z)-lat2ty(n,z)+1;
    if (nx>=1&&ny>=1&&nx<=6&&ny<=6) return z; }
  return 15;
}
boolean fetchTiles(double w, double s, double e, double n) {
  int z=chooseZoom(w,s,e,n);
  int x0=lon2tx(w,z), x1=lon2tx(e,z), y0=lat2ty(n,z), y1=lat2ty(s,z);
  int nx=x1-x0+1, ny=y1-y0+1;
  if (nx<1||ny<1||nx>6||ny>6) return false;
  PImage img=createImage(nx*256, ny*256, RGB);
  for (int ty=y0; ty<=y1; ty++) for (int tx=x0; tx<=x1; tx++) {
    byte[] b=loadBytes("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/"+z+"/"+ty+"/"+tx);
    if (b==null) { satFailed=true; return false; }
    saveBytes(sketchPath("_tile.jpg"), b);
    PImage t=loadImage(sketchPath("_tile.jpg"), "jpg");
    if (t==null||t.width<=0) { satFailed=true; return false; }
    img.copy(t, 0,0,t.width,t.height, (tx-x0)*256,(ty-y0)*256,256,256);
  }
  sat=img; satFailed=false;
  satW=tx2lon(x0,z); satE=tx2lon(x1+1,z); satN=ty2lat(y0,z); satS=ty2lat(y1+1,z);
  return true;
}
void maybeRequestSat() {
  if (offlineProj) return;          // using the planner's cached image, no fetch
  if (!satOn || !originSet) return;

  float fieldW = width-340, fieldH = height-40;
  float halfE = (fieldW/2)/viewScale, halfN = (fieldH/2)/viewScale;
  float visW = viewCE-halfE, visE = viewCE+halfE, visS = viewCN-halfN, visN = viewCN+halfN;
  double w=lonAt(visW), e=lonAt(visE), s=latAt(visS), n=latAt(visN);

  boolean need = (sat == null) || w<satW || e>satE || s<satS || n>satN;
  if (!need || millis()-lastSatReq < 3000) return;               // throttle
  lastSatReq = millis();
  fetchTiles(w, s, e, n);
}

void drawSat() {
  if (sat == null || sat.width <= 0) return;
  float xL = sx((float)((satW-originLon)*mPerDegLon));
  float xR = sx((float)((satE-originLon)*mPerDegLon));
  float yT = sy((float)((satN-originLat)*mPerDegLat));   // north edge -> top
  float yB = sy((float)((satS-originLat)*mPerDegLat));
  noTint();
  image(sat, xL, yT, xR-xL, yB-yT);
}

// ----------------------------------------------------------------------------
//  FIELD RENDER
// ----------------------------------------------------------------------------
void drawField() {
  computeView();

  clip(20, 20, width-340, height-40);   // keep field art out of the HUD panel
  maybeRequestSat();
  boolean haveSat = satOn && sat != null && sat.width > 0;
  if (haveSat) drawSat();

  // metre grid (faint over imagery)
  stroke(38, 44, 54, haveSat ? 70 : 255); strokeWeight(1);
  float gm = gridStep();
  for (float e = ceil((viewCE-30)/gm)*gm; e < viewCE+30; e += gm) line(sx(e), 20, sx(e), height-20);
  for (float n = ceil((viewCN-30)/gm)*gm; n < viewCN+30; n += gm) line(20, sy(n), width-320, sy(n));

  // boundary polygon
  if (boundary.size() >= 3) {
    noFill(); stroke(200, 90, 90); strokeWeight(2);
    beginShape();
    for (double[] b : boundary) { double[] en = toLocal(b[0], b[1]); vertex(sx((float)en[0]), sy((float)en[1])); }
    endShape(CLOSE);
  }

  // path polyline + waypoints
  if (waypoints.size() >= 1) {
    stroke(70, 130, 200); strokeWeight(2); noFill();
    beginShape();
    for (double[] w : waypoints) { double[] en = toLocal(w[0], w[1]); vertex(sx((float)en[0]), sy((float)en[1])); }
    endShape();
    for (int i = 0; i < waypoints.size(); i++) {
      double[] en = toLocal(waypoints.get(i)[0], waypoints.get(i)[1]);
      float x = sx((float)en[0]), y = sy((float)en[1]);
      noFill(); stroke(70,130,200,120); strokeWeight(1); ellipse(x, y, wpRadius*2*viewScale, wpRadius*2*viewScale);
      boolean reached = i < seg;                 // seg = current target index
      boolean current = (i == seg) && !done;
      fill(reached ? color(90,200,140) : current ? color(240,190,60) : color(70,130,200));
      noStroke(); ellipse(x, y, 10, 10);
      if (reached) { fill(20,60,30); textAlign(CENTER,CENTER); text("✓", x, y-1); textAlign(LEFT,BASELINE); }
      fill(150,170,190); text(i, x+8, y-8);
    }
  }

  // base station dot (its own GPS position)
  if (haveBase) {
    double[] en = toLocal(baseLat, baseLon);
    float bx = sx((float)en[0]), by = sy((float)en[1]);
    stroke(60,200,220); strokeWeight(2); noFill(); ellipse(bx, by, 15, 15);
    line(bx-10,by,bx+10,by); line(bx,by-10,bx,by+10);
    noStroke(); fill(60,200,220); text("BASE", bx+9, by-9);
  }

  // rover trail (+ manual GPS offset)
  noFill(); stroke(90, 200, 140, 160); strokeWeight(2);
  beginShape();
  for (float[] p : trail) vertex(sx(p[0]+offE), sy(p[1]+offN));
  endShape();

  // rover (+ manual GPS offset)
  if (fixQual >= 1) {
    double[] en = toLocal(lat, lon);
    float x = sx((float)en[0]+offE), y = sy((float)en[1]+offN);
    // heading line (course, compass deg -> screen: 0=N=up)
    float hx = x + sin(radians(course)) * 26;
    float hy = y - cos(radians(course)) * 26;
    stroke(fixColor()); strokeWeight(3); line(x, y, hx, hy);
    // target bearing (dashed-ish)
    float tx = x + sin(radians(tgtBearing)) * 34;
    float ty = y - cos(radians(tgtBearing)) * 34;
    stroke(230, 200, 80, 180); strokeWeight(1); line(x, y, tx, ty);
    fill(fixColor()); noStroke(); ellipse(x, y, 14, 14);
    fill(255); ellipse(x, y, 5, 5);
  }

  // scale bar
  fill(230); stroke(230); textAlign(LEFT, BOTTOM);
  line(24, height-26, 24 + gm*viewScale, height-26);
  noStroke(); text(nf(gm,0,0) + " m", 24, height-30);

  // imagery attribution / toggle hint
  fill(satOn && sat != null ? color(230,230,230,180) : color(200,160,120,180));
  textAlign(RIGHT, BOTTOM);
  String satMsg;
  if (!satOn)              satMsg = "[m] satellite view";
  else if (offlineProj)    satMsg = "Imagery (c) Esri (cached)   [m] hide";
  else if (sat != null)    satMsg = "Imagery (c) Esri, Maxar   [m] hide";
  else if (satFailed)      satMsg = "imagery needs internet (AP has none)   [m] hide";
  else                     satMsg = "loading imagery...   [m] hide";
  text(satMsg, width-328, height-26);
  textAlign(LEFT, BASELINE);

  noClip();
}

// WASD manual pad, bottom-left over the map. Buttons light up with the held
// keys (or click-and-hold); [g] resume auto, [space] brake hold.
float PAD_X0() { return 16; }
float PAD_TOP() { return height - 182; }
float PAD_W() { return 148; }
boolean overModeBtn() {
  float x0=PAD_X0(), top=PAD_TOP();
  return mouseX>=x0+10 && mouseX<=x0+PAD_W()-10 && mouseY>=top+6 && mouseY<=top+30;
}
void cycleMode() { setMode((driveMode + 1) % 3); }   // AUTO -> MANUAL -> HOLD -> AUTO
void setMode(int m) {
  if      (m==0) { sendCmd("$HOLD,0"); sendCmd("$AUTO"); }          // auto
  else if (m==1) { sendCmd("$HOLD,0"); sendCmd("$MAN,0.00,0.00"); } // manual (idle, WASD drives)
  else           { sendCmd("$HOLD,1"); }                            // hold (brake)
  driveMode = m;   // optimistic; telemetry confirms
}
void drawManualPad() {
  float x0 = PAD_X0(), top = PAD_TOP(), w = PAD_W(), h = 150;
  fill(18, 22, 28, 205); stroke(48,54,64); rect(x0, top, w, h, 8);

  // clickable mode button: cycles AUTO / MANUAL / HOLD
  String mstr = driveMode==2 ? "HOLD" : (driveMode==1 ? "MANUAL" : "AUTO");
  color mcol = driveMode==2 ? color(210,120,120) : (driveMode==1 ? color(230,200,80) : color(90,200,140));
  stroke(70,80,95); fill(overModeBtn() ? lerpColor(mcol,color(255),0.15) : mcol);
  rect(x0+10, top+6, w-20, 24, 5);
  noStroke(); fill(20); textAlign(CENTER, CENTER); text("MODE: " + mstr, x0+w/2, top+18);
  textAlign(LEFT, BASELINE);

  drawKey('W'); drawKey('A'); drawKey('S'); drawKey('D');

  // speed  - / +  (clickable)
  float sy0 = top + h - 48;
  stroke(70,80,95); fill(40,46,56);
  rect(x0+14, sy0, 26, 22, 4); rect(x0+w-40, sy0, 26, 22, 4);
  noStroke(); fill(200); textAlign(CENTER, CENTER);
  text("-", x0+27, sy0+10); text("+", x0+w-27, sy0+10);
  fill(90,200,140); text("SPD " + int(maxPwm), x0+w/2, sy0+10);

  fill(120,135,150); textAlign(CENTER, TOP);
  text("click MODE / speed, or WASD", x0 + w/2, top + h - 18);
  textAlign(LEFT, BASELINE);
}
boolean overSpdMinus() { float x0=PAD_X0(), y=PAD_TOP()+150-48; return mouseX>=x0+14 && mouseX<=x0+40 && mouseY>=y && mouseY<=y+22; }
boolean overSpdPlus()  { float x0=PAD_X0(), w=PAD_W(), y=PAD_TOP()+150-48; return mouseX>=x0+w-40 && mouseX<=x0+w-14 && mouseY>=y && mouseY<=y+22; }
void setSpeed(float v) { maxPwm = constrain(v, 0, 255); sendCmd("$SPD," + int(maxPwm)); }
void mousePressed() {
  if      (overModeBtn())  cycleMode();
  else if (overSpdMinus()) setSpeed(maxPwm - 5);
  else if (overSpdPlus())  setSpeed(maxPwm + 5);
}
void drawKey(char k) {
  boolean on = (k=='W'&&eW)||(k=='A'&&eA)||(k=='S'&&eS)||(k=='D'&&eD);
  float[] p = keyPos(k); float s = 30;
  rectMode(CENTER);
  stroke(70,80,95); fill(on ? color(90,180,200) : color(40,46,56));
  rect(p[0], p[1], s, s, 6);
  fill(on ? color(15,20,25) : color(185,195,205)); noStroke();
  textAlign(CENTER, CENTER); text(k, p[0], p[1]);
  rectMode(CORNER); textAlign(LEFT, BASELINE);
}

float gridStep() {
  float target = 40 / viewScale;   // ~40 px per grid line
  float[] steps = {0.5,1,2,5,10,20,50,100,200};
  for (float s : steps) if (s >= target) return s;
  return 500;
}

// ----------------------------------------------------------------------------
//  HUD
// ----------------------------------------------------------------------------
color fixColor() {
  switch (fixQual) {
    case 4: return color(90, 220, 130);   // RTK fixed
    case 5: return color(230, 200, 80);   // RTK float
    case 2: return color(120, 180, 240);  // DGPS
    case 1: return color(200, 200, 200);  // GPS
    default: return color(150, 90, 90);   // none
  }
}
String fixName() {
  switch (fixQual) { case 4: return "RTK FIXED"; case 5: return "RTK FLOAT";
    case 2: return "DGPS"; case 1: return "GPS (no corr.)"; default: return "NO FIX"; }
}

void drawHud() {
  int px = width - 300, py = 16, w = 284;
  fill(24, 28, 36); stroke(48, 54, 64); rect(px, py, w, height - 32, 8);
  int x = px + 16, y = py + 30;

  fill(230); textSize(16); text("PiSat RTK Telemetry", x, y); y += 10;
  textSize(13);

  boolean live = connected && (millis() - lastRxMs < 2000);
  drawKV(x, y+=24, "Link", live ? "CONNECTED " + ROVER_HOST : "waiting...",
         live ? color(90,200,140) : color(210,150,90));
  drawKV(x, y+=20, "Bad lines", "" + badLines,
         badLines == 0 ? color(120,135,150) : color(210,150,90));
  boolean cmdUp = cmd != null && cmd.active();
  drawKV(x, y+=20, "Cmd link :8889", cmdUp ? "up" : "DOWN (no control)",
         cmdUp ? color(90,200,140) : color(210,120,120));

  // fix quality banner
  y += 18;
  fill(fixColor()); noStroke(); rect(x, y, w-32, 30, 5);
  fill(20); textSize(15); textAlign(CENTER, CENTER);
  text(fixName() + "   (" + sats + " sats)", x + (w-32)/2, y + 15);
  textAlign(LEFT, BASELINE); textSize(13); y += 30;

  drawKV(x, y+=26, "Latitude",  nfd(lat, 7), color(210));
  drawKV(x, y+=22, "Longitude", nfd(lon, 7), color(210));
  drawKV(x, y+=22, "HDOP", nf(hdop,0,1), hdop < 2 ? color(90,200,140) : color(210,180,90));
  drawKV(x, y+=22, "Speed", nf(speed,0,2) + " m/s", color(210));
  drawKV(x, y+=22, "Course", nf(course,0,1) + " deg", color(210));

  y += 10; stroke(48,54,64); line(x, y, x+w-32, y);
  float distWp = -1;
  if (fixQual>=1 && waypoints.size()>0) {
    int ti = min(seg, waypoints.size()-1);      // seg = current target waypoint
    double[] we = toLocal(waypoints.get(ti)[0], waypoints.get(ti)[1]);
    double[] rv = toLocal(lat, lon);
    distWp = dist((float)(rv[0]+offE),(float)(rv[1]+offN),(float)we[0],(float)we[1]);
  }
  drawKV(x, y+=26, "Target wp", (done ? waypoints.size() : min(seg, max(0,waypoints.size()-1)))
         + " / " + max(0, waypoints.size()-1)
         + (distWp>=0 ? "   " + nf(distWp,0,1) + " m" : ""), color(210));
  drawKV(x, y+=22, "Target brg", nf(tgtBearing,0,1) + " deg", color(210));
  drawKV(x, y+=22, "Heading err", nf(hdgErr,0,1) + " deg",
         abs(hdgErr) < 10 ? color(90,200,140) : color(210,150,90));
  drawKV(x, y+=22, "Path", done ? "COMPLETE" : "driving",
         done ? color(90,200,140) : color(210));

  y += 10; stroke(48,54,64); line(x, y, x+w-32, y);
  // differential-correction link health
  boolean corr = rtcmAge < 5000;
  drawKV(x, y+=26, "Corr age", corr ? rtcmAge + " ms" : "no corrections",
         corr ? color(90,200,140) : color(210,120,120));
  drawKV(x, y+=22, "PWM R / L", nf(pwmR,0,0) + " / " + nf(pwmL,0,0)
         + "  (max " + nf(maxPwm,0,0) + ")", color(210));
  drawKV(x, y+=22, "Wheel Hz R/L", nf(spdHzR,0,1) + " / " + nf(spdHzL,0,1), color(210));
  drawKV(x, y+=22, "Wheel rpm R/L", int(spdHzR/PPR*60) + " / " + int(spdHzL/PPR*60),
         (spdHzR>0||spdHzL>0) ? color(120,180,240) : color(150));
  drawKV(x, y+=22, "Speed [ / ]", nf(maxPwm,0,0) + " / 255", color(90,200,140));
  drawKV(x, y+=22, "PiSat batt", vbat>1 ? nf(vbat,0,2)+" V" : "no batt (ok)",
         vbat>1 ? color(210) : color(150));
  drawKV(x, y+=22, "GPS offset E/N", nf(offE,0,1) + " / " + nf(offN,0,1) + " m",
         (offE!=0||offN!=0) ? color(230,200,80) : color(150));
  fill(120,135,150); text("arrows nudge offset, [o] reset", x, y+=16);

  // wheel PWM bars
  y += 12;
  drawBar(x, y, "R", pwmR / 255.0);
  drawBar(x + (w-32)/2 + 6, y, "L", pwmL / 255.0);
}

void drawKV(int x, int y, String k, String v, color cv) {
  fill(120,135,150); text(k, x, y);
  fill(cv); textAlign(RIGHT, BASELINE); text(v, x + 268, y); textAlign(LEFT, BASELINE);
}
void drawBar(int x, int y, String label, float frac) {
  frac = constrain(frac, 0, 1);
  fill(120,135,150); text(label, x, y+12);
  int bx = x + 16, bw = 110;
  noFill(); stroke(60,66,76); rect(bx, y, bw, 12, 3);
  noStroke(); fill(90,180,200); rect(bx, y, bw*frac, 12, 3);
}
void legendDot(int x, int y, color c, String s) {
  fill(c); noStroke(); ellipse(x+5, y-4, 10, 10);
  fill(150); text(s, x+16, y);
}

// double -> fixed-decimal string (Processing's nf() is float-only)
String nfd(double v, int dec) {
  return String.format("%." + dec + "f", v);
}

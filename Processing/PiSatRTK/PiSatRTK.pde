// ============================================================================
//  PiSat RTK  --  rover telemetry & path visualiser  (Processing 4.x)
//
//  Connects over TCP to the rover ESP32's telemetry server and draws the rover
//  driving its GPS waypoint path in a local metre frame, with a live HUD:
//  RTK fix quality, position, satellites, HDOP, speed/course, cross-track
//  heading error, correction-link (RTCM) age and per-wheel PWM.
//
//  SETUP
//    1. Flash the firmware and power up base + rover (see RTK_SETUP_INSTRUCTIONS.txt).
//    2. Join the WiFi network "PiSat-RTK" (password pisat-rtk-1234) on this computer.
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
String ROVER_HOST = "192.168.4.1";
int    ROVER_PORT = 8888;
Client rover;
boolean connected = false;
int    lastRxMs   = 0;
int    lastConnectAttempt = 0;

// ---- world model ----
boolean originSet = false;
double  originLat = 0, originLon = 0;
double  mPerDegLat = 111320.0, mPerDegLon = 111320.0;

ArrayList<double[]> waypoints = new ArrayList<double[]>();  // {lat,lon}
ArrayList<double[]> boundary  = new ArrayList<double[]>();
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
  connect();
}

void connect() {
  lastConnectAttempt = millis();
  try {
    rover = new Client(this, ROVER_HOST, ROVER_PORT);
    connected = rover.active();
  } catch (Exception e) {
    connected = false;
  }
}

// East/North metres from lat/lon in the rover's local frame.
double[] toLocal(double la, double lo) {
  return new double[] { (lo - originLon) * mPerDegLon, (la - originLat) * mPerDegLat };
}

void draw() {
  background(17, 20, 26);
  pumpNetwork();

  if (originSet) { drawField(); }
  drawHud();

  // auto-reconnect
  if ((rover == null || !rover.active()) && millis() - lastConnectAttempt > 2000) {
    connected = false;
    connect();
  }
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

void handleLine(String line) {
  String[] f = split(line, ',');
  if (f.length == 0) return;

  if (f[0].equals("T") && f.length >= 16) {
    fixQual    = int(f[2]);
    lat        = Double.parseDouble(f[3]);
    lon        = Double.parseDouble(f[4]);
    course     = float(f[5]);
    speed      = float(f[6]);
    sats       = int(f[7]);
    hdop       = float(f[8]);
    seg        = int(f[9]);
    tgtBearing = float(f[10]);
    hdgErr     = float(f[11]);
    done       = int(f[12]) != 0;
    rtcmAge    = Long.parseLong(f[13]);
    pwmR       = float(f[14]);
    pwmL       = float(f[15]);
    if (f.length >= 18) { spdHzR = float(f[16]); spdHzL = float(f[17]); }
    fixMs      = millis();
    if (fixQual >= 1 && originSet) {
      double[] en = toLocal(lat, lon);
      float[] p = new float[] { (float)en[0], (float)en[1] };
      if (trail.isEmpty() || dist(p[0], p[1], trail.get(trail.size()-1)[0], trail.get(trail.size()-1)[1]) > 0.05)
        trail.add(p);
      if (trail.size() > 4000) trail.remove(0);
    }
  } else if (f[0].equals("O") && f.length >= 3) {
    setOrigin(Double.parseDouble(f[1]), Double.parseDouble(f[2]));
  } else if (f[0].equals("W") && f.length >= 4) {
    ensureSize(waypoints, int(f[1]));
    waypoints.set(int(f[1]), new double[]{ Double.parseDouble(f[2]), Double.parseDouble(f[3]) });
  } else if (f[0].equals("B") && f.length >= 4) {
    ensureSize(boundary, int(f[1]));
    boundary.set(int(f[1]), new double[]{ Double.parseDouble(f[2]), Double.parseDouble(f[3]) });
  } else if (f[0].equals("M") && f.length >= 3) {
    maxPwm   = float(f[1]);
    wpRadius = float(f[2]);
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

// ----------------------------------------------------------------------------
//  FIELD RENDER
// ----------------------------------------------------------------------------
void drawField() {
  computeView();

  // metre grid
  stroke(38, 44, 54); strokeWeight(1);
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
      fill(i <= seg ? color(90,200,140) : color(70,130,200)); noStroke(); ellipse(x, y, 10, 10);
      fill(150,170,190); text(i, x+8, y-8);
    }
  }

  // rover trail
  noFill(); stroke(90, 200, 140, 160); strokeWeight(2);
  beginShape();
  for (float[] p : trail) vertex(sx(p[0]), sy(p[1]));
  endShape();

  // rover
  if (fixQual >= 1) {
    double[] en = toLocal(lat, lon);
    float x = sx((float)en[0]), y = sy((float)en[1]);
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
  fill(150,170,190); noStroke(); textAlign(LEFT, BOTTOM);
  line(24, height-26, 24 + gm*viewScale, height-26);
  text(nf(gm,0,0) + " m", 24, height-30);
  textAlign(LEFT, BASELINE);
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
  drawKV(x, y+=26, "Segment", seg + " / " + max(0, waypoints.size()-1), color(210));
  drawKV(x, y+=22, "Target brg", nf(tgtBearing,0,1) + " deg", color(210));
  drawKV(x, y+=22, "Heading err", nf(hdgErr,0,1) + " deg",
         abs(hdgErr) < 10 ? color(90,200,140) : color(210,150,90));
  drawKV(x, y+=22, "Path", done ? "COMPLETE" : "driving",
         done ? color(90,200,140) : color(210));

  y += 10; stroke(48,54,64); line(x, y, x+w-32, y);
  // RTCM correction link health
  boolean corr = rtcmAge < 5000;
  drawKV(x, y+=26, "RTCM age", corr ? rtcmAge + " ms" : "no corrections",
         corr ? color(90,200,140) : color(210,120,120));
  drawKV(x, y+=22, "PWM R / L", nf(pwmR,0,0) + " / " + nf(pwmL,0,0)
         + "  (max " + nf(maxPwm,0,0) + ")", color(210));
  drawKV(x, y+=22, "Wheel R / L", nf(spdHzR,0,1) + " / " + nf(spdHzL,0,1) + " Hz",
         (spdHzR > 0 || spdHzL > 0) ? color(120,180,240) : color(150));

  // wheel bars
  y += 16;
  drawBar(x, y, "R", pwmR / 255.0);
  drawBar(x + (w-32)/2 + 6, y, "L", pwmL / 255.0);

  // legend
  y = height - 120;
  fill(160); text("Fix colours:", x, y);
  legendDot(x, y+18, color(90,220,130), "RTK fixed (cm)");
  legendDot(x, y+38, color(230,200,80), "RTK float");
  legendDot(x, y+58, color(120,180,240), "DGPS (sub-m)");
  legendDot(x, y+78, color(200,200,200), "GPS only");
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

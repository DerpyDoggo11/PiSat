// ============================================================================
//  PiSat PLANNER  --  path + boundary editor over satellite imagery
//  (Processing 4.x)
//
//  Plan your field ONLINE here, then it caches the satellite image + your path +
//  boundary into ../shared/, which the runtime sketch (PiSatRTK) loads OFFLINE.
//
//  NEEDS INTERNET while planning (Esri World Imagery, free / no API key).
//
//  CONTROLS
//    left-drag ............ pan the map        mouse wheel / + - .... zoom
//    left-click ........... add a point (in the current mode)
//    [w] path mode   [b] boundary mode
//    [z] undo last point (current mode)        [x] delete nearest point
//    [c] clear current mode                    [C] clear everything
//    [s] SAVE project (caches imagery -> ../shared/)   [l] load project
//
//  Path = ordered waypoints the rover drives. Boundary = polygon; the rover
//  STOPS if it leaves it (>= 3 points to be active).
// ============================================================================

// ---- initial view (edit to your field, or just pan/zoom to it) --------------
double viewLat = 48.7597082, viewLon = -122.3620117;
float  mPerPx  = 0.20;                 // zoom: metres per pixel (smaller = closer)

int    MODE_PATH = 0, MODE_BND = 1, MODE_BASE = 2, mode = MODE_PATH;
ArrayList<double[]> wp  = new ArrayList<double[]>();   // {lat, lon}
ArrayList<double[]> bnd = new ArrayList<double[]>();
double baseLat = 0, baseLon = 0; boolean haveBase = false;   // base station point

// ---- satellite imagery (Esri World Imagery) --------------------------------
PImage  sat = null, satPending = null;
double  satW=0, satE=0, satS=0, satN=0;      // displayed tile bounds
double  pW=0, pE=0, pS=0, pN=0;              // pending request bounds
int     lastReq = 0;
boolean satFailed = false;

// ---- save workflow ----------------------------------------------------------
boolean pendingSave = false;
String  status = "";
int     statusMs = 0;

// ---- layout / interaction ---------------------------------------------------
int   fieldX0, fieldY0, fieldW, fieldH;
double vW, vE, vS, vN;                        // current view bounds (lon/lat)
boolean dragging = false;
int   pressX, pressY;
PFont mono;

String SHARED = "../shared/";                 // shared with the PiSatRTK sketch

void settings() { size(1180, 760); }

void setup() {
  surface.setTitle("PiSat Planner  --  path & boundary editor");
  mono = createFont("Consolas", 13);
  textFont(mono);
  layout();
}
void layout() { fieldX0 = 20; fieldY0 = 20; fieldW = width - 340; fieldH = height - 40; }

double mPerDegLat() { return 111320.0; }
double mPerDegLon() { return 111320.0 * Math.cos(Math.toRadians(viewLat)); }

void computeBounds() {
  double halfWm = (fieldW/2.0) * mPerPx, halfHm = (fieldH/2.0) * mPerPx;
  double dLat = halfHm / mPerDegLat(), dLon = halfWm / mPerDegLon();
  vW = viewLon - dLon; vE = viewLon + dLon;
  vS = viewLat - dLat; vN = viewLat + dLat;
}
float lon2x(double lon) { return (float)(fieldX0 + (lon - vW) / (vE - vW) * fieldW); }
float lat2y(double lat) { return (float)(fieldY0 + (vN - lat) / (vN - vS) * fieldH); }
double x2lon(float x)   { return vW + (x - fieldX0) / (double)fieldW * (vE - vW); }
double y2lat(float y)   { return vN - (y - fieldY0) / (double)fieldH * (vN - vS); }

void draw() {
  background(17, 20, 26);
  computeBounds();

  clip(fieldX0, fieldY0, fieldW, fieldH);
  maybeRequestSat();
  drawSat();
  drawBoundary();
  drawPath();
  drawBase();
  noClip();

  drawHud();
  handleSave();
}

// ---- imagery (Esri World Imagery XYZ TILES -- the reliable endpoint) --------
// The MapServer/export op 500s for small bboxes; the tile endpoint never does.
// We fetch the covering tiles with loadBytes (no URL-extension check), stitch
// them, and record the stitched image's exact geo-bounds. Synchronous + throttled.
int  lon2tx(double lon, int z) { return (int)Math.floor((lon+180.0)/360.0*(1L<<z)); }
int  lat2ty(double lat, int z) { double r=Math.toRadians(lat);
  return (int)Math.floor((1-Math.log(Math.tan(r)+1/Math.cos(r))/Math.PI)/2*(1L<<z)); }
double tx2lon(int x, int z) { return x/(double)(1L<<z)*360.0-180.0; }
double ty2lat(int y, int z) { double m=Math.PI-2.0*Math.PI*y/(1L<<z); return Math.toDegrees(Math.atan(Math.sinh(m))); }

int chooseZoom(double w, double s, double e, double n) {
  for (int z=19; z>=15; z--) {   // World_Imagery cache guaranteed to z19; deeper = "Map Not Available"
    int nx=lon2tx(e,z)-lon2tx(w,z)+1, ny=lat2ty(s,z)-lat2ty(n,z)+1;
    if (nx>=1 && ny>=1 && nx<=6 && ny<=6) return z;
  }
  return 15;
}
// Fetch + stitch tiles covering [w,s,e,n]; on success set sat + satW/E/S/N.
boolean fetchTiles(double w, double s, double e, double n) {
  int z = chooseZoom(w,s,e,n);
  int x0=lon2tx(w,z), x1=lon2tx(e,z), y0=lat2ty(n,z), y1=lat2ty(s,z);   // y0=north
  int nx=x1-x0+1, ny=y1-y0+1;
  if (nx<1||ny<1||nx>6||ny>6) return false;
  PImage img = createImage(nx*256, ny*256, RGB);
  for (int ty=y0; ty<=y1; ty++) for (int tx=x0; tx<=x1; tx++) {
    String url = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/"
               + z + "/" + ty + "/" + tx;
    byte[] b = loadBytes(url);
    if (b == null) { satFailed = true; return false; }
    saveBytes(sketchPath("_tile.jpg"), b);
    PImage t = loadImage(sketchPath("_tile.jpg"), "jpg");
    if (t == null || t.width <= 0) { satFailed = true; return false; }
    img.copy(t, 0,0,t.width,t.height, (tx-x0)*256,(ty-y0)*256,256,256);
  }
  sat = img; satFailed = false;
  satW = tx2lon(x0, z); satE = tx2lon(x1+1, z);       // tile grid geo-extent
  satN = ty2lat(y0, z); satS = ty2lat(y1+1, z);
  return true;
}
void maybeRequestSat() {
  boolean need = (sat == null) || vW<satW || vE>satE || vS<satS || vN>satN;
  if (!need || millis()-lastReq < 1200 || pendingSave) return;
  lastReq = millis();
  fetchTiles(vW, vS, vE, vN);
}
void drawSat() {
  if (sat == null || sat.width <= 0) { fill(30,34,42); noStroke(); rect(fieldX0,fieldY0,fieldW,fieldH); return; }
  noTint();
  image(sat, lon2x(satW), lat2y(satN), lon2x(satE)-lon2x(satW), lat2y(satS)-lat2y(satN));
}

// ---- overlays --------------------------------------------------------------
void drawBoundary() {
  if (bnd.isEmpty()) return;
  stroke(220, 90, 90); strokeWeight(2);
  fill(220, 90, 90, 30);
  beginShape();
  for (double[] p : bnd) vertex(lon2x(p[1]), lat2y(p[0]));
  if (bnd.size() >= 3) endShape(CLOSE); else endShape();
  noStroke(); fill(220,90,90);
  for (double[] p : bnd) ellipse(lon2x(p[1]), lat2y(p[0]), 8, 8);
}
void drawPath() {
  if (!wp.isEmpty()) {
    noFill(); stroke(90, 200, 140); strokeWeight(2);
    beginShape();
    for (double[] p : wp) vertex(lon2x(p[1]), lat2y(p[0]));
    endShape();
  }
  for (int i = 0; i < wp.size(); i++) {
    float x = lon2x(wp.get(i)[1]), y = lat2y(wp.get(i)[0]);
    noStroke(); fill(90, 200, 140); ellipse(x, y, 11, 11);
    fill(255); textAlign(CENTER, CENTER); text(i, x, y-11);
  }
  textAlign(LEFT, BASELINE);
}
void drawBase() {
  if (!haveBase) return;
  float x = lon2x(baseLon), y = lat2y(baseLat);
  stroke(60,200,220); strokeWeight(2); noFill();
  ellipse(x, y, 16, 16); line(x-11,y,x+11,y); line(x,y-11,x,y+11);   // survey cross
  noStroke(); fill(60,200,220); textAlign(CENTER,BOTTOM); text("BASE", x, y-10);
  textAlign(LEFT, BASELINE);
}

// ---- HUD -------------------------------------------------------------------
void drawHud() {
  int px = width-300, py=16, w=284;
  fill(24,28,36); stroke(48,54,64); rect(px, py, w, height-32, 8);
  int x=px+16, y=py+34;
  fill(230); textSize(16); text("PiSat Planner", x, y); textSize(13);

  y+=14;
  fill(mode==MODE_PATH?color(90,200,140):color(90,110,130)); noStroke();
  rect(x, y, (w-40)/2, 26, 5);
  fill(mode==MODE_BND?color(220,120,120):color(90,110,130));
  rect(x+(w-40)/2+8, y, (w-40)/2, 26, 5);
  fill(20); textAlign(CENTER,CENTER);
  text("PATH [w]", x+(w-40)/4, y+13);
  text("BOUNDARY [b]", x+(w-40)*3/4+8, y+13);
  textAlign(LEFT,BASELINE); y+=44;

  fill(120,135,150); text("Waypoints", x, y); fill(90,200,140); textAlign(RIGHT,BASELINE); text(""+wp.size(), x+268, y); textAlign(LEFT,BASELINE); y+=22;
  fill(120,135,150); text("Boundary pts", x, y); fill(220,120,120); textAlign(RIGHT,BASELINE); text(bnd.size()+(bnd.size()>=3?" (active)":" (need 3)"), x+268, y); textAlign(LEFT,BASELINE); y+=22;
  fill(120,135,150); text("Base [g]", x, y); fill(60,200,220); textAlign(RIGHT,BASELINE); text(haveBase?"set":"none", x+268, y); textAlign(LEFT,BASELINE); y+=22;
  if (haveBase && mode==MODE_BASE) {
    fill(60,200,220); text("-DBASE_REF_LAT=" + String.format("%.7f",baseLat), x, y); y+=18;
    text("-DBASE_REF_LON=" + String.format("%.7f",baseLon), x, y); y+=4;
  }

  y+=6; stroke(48,54,64); line(x,y,x+w-32,y); y+=8;
  fill(150,170,190);
  String[] help = {
    "left-drag: pan    wheel/+ -: zoom",
    "left-click: add point (current mode)",
    "[w] path  [b] boundary  [g] base",
    "[z] undo   [x] delete nearest",
    "[c] clear mode   [C] clear all",
    "[s] SAVE project (caches imagery)",
    "[l] load project"
  };
  for (String h : help) { text(h, x, y); y += 20; }

  // cursor lat/lon
  y += 8; fill(120,135,150);
  if (mouseX>=fieldX0 && mouseX<fieldX0+fieldW && mouseY>=fieldY0 && mouseY<fieldY0+fieldH)
    text(String.format("%.7f, %.7f", y2lat(mouseY), x2lon(mouseX)), x, y);

  // status line
  if (millis()-statusMs < 4000 && status.length()>0) {
    fill(90,200,140); text(status, x, height-30);
  }
  if (satFailed) { fill(210,150,90); text("imagery needs internet", x, height-50); }
}

// ---- interaction -----------------------------------------------------------
void mousePressed() { pressX=mouseX; pressY=mouseY; dragging=false; }
void mouseDragged() {
  if (abs(mouseX-pressX)+abs(mouseY-pressY) > 3) dragging = true;
  // pan by the mouse delta (convert px -> degrees)
  double dLon = (mouseX-pmouseX) / (double)fieldW * (vE-vW);
  double dLat = (mouseY-pmouseY) / (double)fieldH * (vN-vS);
  viewLon -= dLon; viewLat += dLat;
}
void mouseReleased() {
  if (dragging) return;
  if (mouseX<fieldX0 || mouseX>=fieldX0+fieldW || mouseY<fieldY0 || mouseY>=fieldY0+fieldH) return;
  double la = y2lat(mouseY), lo = x2lon(mouseX);
  if (mode==MODE_BASE) { baseLat=la; baseLon=lo; haveBase=true; }
  else (mode==MODE_PATH ? wp : bnd).add(new double[]{la, lo});
}
void mouseWheel(processing.event.MouseEvent e) {
  mPerPx *= (e.getCount() > 0) ? 1.2 : (1/1.2);
  mPerPx = constrain(mPerPx, 0.02f, 5.0f);
}
void keyPressed() {
  if      (key=='w') mode=MODE_PATH;
  else if (key=='b') mode=MODE_BND;
  else if (key=='g') mode=MODE_BASE;
  else if (key=='z') { ArrayList<double[]> a = (mode==MODE_PATH?wp:bnd); if(!a.isEmpty()) a.remove(a.size()-1); }
  else if (key=='x') deleteNearest();
  else if (key=='c') { if (mode==MODE_BASE) haveBase=false; else (mode==MODE_PATH?wp:bnd).clear(); }
  else if (key=='C') { wp.clear(); bnd.clear(); haveBase=false; }
  else if (key=='s') startSave();
  else if (key=='l') loadProject();
  else if (key=='+' || key=='=') mPerPx = constrain(mPerPx/1.2f, 0.02f, 5.0f);
  else if (key=='-' || key=='_') mPerPx = constrain(mPerPx*1.2f, 0.02f, 5.0f);
  else if (keyCode==LEFT)  viewLon -= (vE-vW)*0.1;
  else if (keyCode==RIGHT) viewLon += (vE-vW)*0.1;
  else if (keyCode==UP)    viewLat += (vN-vS)*0.1;
  else if (keyCode==DOWN)  viewLat -= (vN-vS)*0.1;
}
void deleteNearest() {
  ArrayList<double[]> a = (mode==MODE_PATH?wp:bnd);
  int best=-1; float bd=18;
  for (int i=0;i<a.size();i++){ float d=dist(mouseX,mouseY,lon2x(a.get(i)[1]),lat2y(a.get(i)[0])); if(d<bd){bd=d;best=i;} }
  if (best>=0) a.remove(best);
}

// ---- save / load -----------------------------------------------------------
void startSave() {
  if (wp.isEmpty() && bnd.isEmpty() && !haveBase) { setStatus("nothing to save"); return; }
  // auto-fit the view to enclose every point (+ margin), then fetch that tile.
  double minLa=1e9,maxLa=-1e9,minLo=1e9,maxLo=-1e9;
  for (double[] p : wp)  { minLa=Math.min(minLa,p[0]);maxLa=Math.max(maxLa,p[0]);minLo=Math.min(minLo,p[1]);maxLo=Math.max(maxLo,p[1]); }
  for (double[] p : bnd) { minLa=Math.min(minLa,p[0]);maxLa=Math.max(maxLa,p[0]);minLo=Math.min(minLo,p[1]);maxLo=Math.max(maxLo,p[1]); }
  if (haveBase) { minLa=Math.min(minLa,baseLat);maxLa=Math.max(maxLa,baseLat);minLo=Math.min(minLo,baseLon);maxLo=Math.max(maxLo,baseLon); }
  double mLa=(maxLa-minLa)*0.15+0.00005, mLo=(maxLo-minLo)*0.15+0.00005;
  minLa-=mLa;maxLa+=mLa;minLo-=mLo;maxLo+=mLo;
  viewLat=(minLa+maxLa)/2; viewLon=(minLo+maxLo)/2;
  double wm=(maxLo-minLo)*mPerDegLon(), hm=(maxLa-minLa)*mPerDegLat();
  mPerPx = constrain((float)Math.max(wm/fieldW, hm/fieldH), 0.02f, 5.0f);
  computeBounds();
  setStatus("fetching imagery to cache...");
  if (!fetchTiles(vW, vS, vE, vN)) { setStatus("SAVE failed: no imagery (need internet)"); return; }
  writeProject();
}
void handleSave() { }   // fetch is synchronous now; kept so draw()'s call is valid
void writeProject() {
  try {
    java.io.File dir = new java.io.File(sketchPath(SHARED));
    if (!dir.exists()) dir.mkdirs();
    if (sat != null && sat.width > 0) sat.save(sketchPath(SHARED + "project.jpg"));
    ArrayList<String> out = new ArrayList<String>();
    out.add("# PiSat project");
    out.add("bounds " + satW + " " + satS + " " + satE + " " + satN);
    out.add("image project.jpg");
    if (haveBase) out.add("base " + baseLat + " " + baseLon);
    for (double[] p : wp)  out.add("wp "  + p[0] + " " + p[1]);
    for (double[] p : bnd) out.add("bnd " + p[0] + " " + p[1]);
    saveStrings(sketchPath(SHARED + "project.pisat"), out.toArray(new String[0]));
    setStatus("SAVED to ../shared/  (" + wp.size() + " wp, " + bnd.size() + " bnd)");
  } catch (Exception ex) { setStatus("SAVE error: " + ex.getMessage()); }
}
void loadProject() {
  String[] lines = loadStrings(sketchPath(SHARED + "project.pisat"));
  if (lines == null) { setStatus("no project.pisat in ../shared/"); return; }
  wp.clear(); bnd.clear(); haveBase=false;
  for (String ln : lines) {
    String[] t = splitTokens(trim(ln), " ");
    if (t.length==0) continue;
    if (t[0].equals("base") && t.length>=3) { baseLat=Double.parseDouble(t[1]); baseLon=Double.parseDouble(t[2]); haveBase=true; }
    else if (t[0].equals("wp")  && t.length>=3) wp.add(new double[]{Double.parseDouble(t[1]),Double.parseDouble(t[2])});
    else if (t[0].equals("bnd") && t.length>=3) bnd.add(new double[]{Double.parseDouble(t[1]),Double.parseDouble(t[2])});
    else if (t[0].equals("bounds") && t.length>=5) {
      double w=Double.parseDouble(t[1]),s=Double.parseDouble(t[2]),e=Double.parseDouble(t[3]),n=Double.parseDouble(t[4]);
      viewLat=(s+n)/2; viewLon=(w+e)/2;
      mPerPx=constrain((float)Math.max((e-w)*mPerDegLon()/fieldW,(n-s)*mPerDegLat()/fieldH),0.02f,5.0f);
    }
  }
  sat=null; setStatus("loaded " + wp.size() + " wp, " + bnd.size() + " bnd");
}
void setStatus(String s) { status=s; statusMs=millis(); }

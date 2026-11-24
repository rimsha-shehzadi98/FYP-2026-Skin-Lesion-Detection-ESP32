/* ============================================================
   Acne & Skin Disease Detection - FYP Dashboard V5 Final
   Government College University Faisalabad 2022-2026
   Supervisor: Dr. Usama Ahmed
   Iqra Waheed (221982) | Rimsha Shehzadi (222027)
   ============================================================ */

#include <FYP_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>

const char* ssid     = "Ufone-LTE-39F05";
const char* password = "29thd033";

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS   320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS   240
#define EI_CAMERA_FRAME_BYTE_SIZE         3
#define MAX_CAPTURES                      12

struct CaptureResult {
  String   label;
  float    confidence;
  bool     hasDetection;
  String   timestamp;
  uint8_t* jpegData;
  size_t   jpegLen;
};

static bool is_initialised = false;
static bool debug_nn       = false;
uint8_t*    snapshot_buf;
bool isRunning   = false;
bool doCapture   = false;
bool limitAlert  = false;

CaptureResult captures[MAX_CAPTURES];
int captureCount = 0;

WebServer server(80);

static camera_config_t camera_config = {
    .pin_pwdn=PWDN_GPIO_NUM,.pin_reset=RESET_GPIO_NUM,.pin_xclk=XCLK_GPIO_NUM,
    .pin_sscb_sda=SIOD_GPIO_NUM,.pin_sscb_scl=SIOC_GPIO_NUM,
    .pin_d7=Y9_GPIO_NUM,.pin_d6=Y8_GPIO_NUM,.pin_d5=Y7_GPIO_NUM,.pin_d4=Y6_GPIO_NUM,
    .pin_d3=Y5_GPIO_NUM,.pin_d2=Y4_GPIO_NUM,.pin_d1=Y3_GPIO_NUM,.pin_d0=Y2_GPIO_NUM,
    .pin_vsync=VSYNC_GPIO_NUM,.pin_href=HREF_GPIO_NUM,.pin_pclk=PCLK_GPIO_NUM,
    .xclk_freq_hz=20000000,.ledc_timer=LEDC_TIMER_0,.ledc_channel=LEDC_CHANNEL_0,
    .pixel_format=PIXFORMAT_JPEG,.frame_size=FRAMESIZE_QVGA,
    .jpeg_quality=12,.fb_count=1,.fb_location=CAMERA_FB_IN_PSRAM,
    .grab_mode=CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t w,uint32_t h,uint8_t* buf);
static int ei_camera_get_data(size_t offset,size_t length,float* out_ptr);

static const char htmlPage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Acne & Skin Disease Detection - FYP</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box;margin:0;padding:0;}
html,body{height:100%;font-family:'Segoe UI',Arial,sans-serif;background:#EEF4FB;color:#1a2e4a; font-size:22px}
a{color:#1565c0;text-decoration:none;}

/* HEADER */
.hdr{background:linear-gradient(135deg,#1a3a8f 0%,#2255cc 100%);padding:10px 22px;display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:200;box-shadow:0 2px 12px rgba(26,58,143,0.3);}
.hdr-left h1{color:white;font-size:20px;font-weight:600;}
.hdr-left p{color:rgba(255,255,255,0.5);font-size:13px;margin-top:1px;}
.hdr-right{display:flex;align-items:center;gap:8px;}
.status-pill{border-radius:20px;padding:3px 12px;font-size:9px;font-weight:600;letter-spacing:0.4px;}
.pill-run{background:#e8f5e9;color:#2e7d32;}
.pill-pause{background:#fce4ec;color:#c62828;}

/* MAIN: fills viewport height minus header */
.main{
  padding:14px 14px 14px 90px;
  display:grid;
  grid-template-columns:1fr 60px 340px;
  gap:12px;
  align-items:stretch;
  height:calc(100vh - 56px);
  align-items:stretch;
  overflow-y:auto;
  overflow:hidden;
}

/* WHITE PANEL */
.panel{background:white;border-radius:13px;border:1px solid #c5d8f0;padding:20px;box-shadow:0 1px 10px rgba(26,58,143,0.07);display:flex;flex-direction:column;justify-content:center}
.panel-title{font-size:14px;font-weight:700;color:#1a3a8f;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;flex-shrink:0;}

/* CAMERA GRID
   4 cols, rows auto — camera spans col2-3 row2-3
   All slots equal: 1fr rows. Grid fills panel height.
*/
.cam-grid{
  display:grid;
  grid-template-columns:repeat(4,110px);
  grid-template-rows:110px 99px 99px 110px;
  gap:8px;
  margin:auto;
}

/* IMAGE SLOTS */
.slot{border-radius:7px;background:#EEF4FB;border:1.5px dashed #c5d8f0;display:flex;flex-direction:column;align-items:center;justify-content:center;cursor:pointer;overflow:hidden;position:relative;transition:transform 0.18s,box-shadow 0.18s;font-size:15px;color:#b0bec5;}
.slot:hover{transform:scale(1.04);box-shadow:0 3px 10px rgba(26,58,143,0.15);}
.slot.filled{border-style:solid;}
.slot.bcc{background:#fce4ec;border-color:#333;}
.slot.nevus{background:#e3f2fd;border-color:#333;}
.slot.none{background:#f5f5f5;border-color:#333;}
.slot img{width:100%;height:100%;object-fit:cover;display:block;}
.slot-badge{position:absolute;bottom:0;left:0;right:0;font-size:11px;font-weight:700;padding:2px 0;text-align:center;background:rgba(0,0,0,0.42);color:white;opacity:0;transition:opacity 0.3s;}
.slot:hover .slot-badge{opacity:1;}
.slot-badge.bcc{background:rgba(20,20,20,0.88);}
.slot-badge.nevus{background:rgba(20,20,20,0.88);}

/* LIVE CAMERA: col2-3 row2-3 */
.live-cam{
  grid-column:2/4;grid-row:2/4;
  border-radius:10px;background:linear-gradient(145deg,#0d1f5c,#1a3a8f);
  border:2px solid #2255cc;display:flex;flex-direction:column;
  align-items:center;justify-content:center;position:relative;overflow:hidden;
  box-shadow:0 3px 16px rgba(26,58,143,0.4);
}
.live-cam img{width:100%;height:100%;object-fit:cover;display:block;}
.live-badge{position:absolute;top:6px;right:7px;background:#e53935;color:white;font-size:6px;padding:2px 6px;border-radius:8px;display:flex;align-items:center;gap:2px;}
.live-dot{width:4px;height:4px;background:white;border-radius:50%;animation:blink 1s infinite;}
.live-info{position:absolute;bottom:6px;left:7px;background:rgba(0,0,0,0.38);color:rgba(255,255,255,0.75);font-size:6px;padding:2px 6px;border-radius:8px;}
.cam-placeholder{display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px;}
.cam-lbl{color:rgba(255,255,255,0.4);font-size:7px;}
@keyframes blink{0%,100%{opacity:1;}50%{opacity:0.3;}}

/* BUTTONS COLUMN */
.btn-col{display:flex;flex-direction:column;gap:16px;align-items:center;justify-content:center;height:100%;}
.cbtn-wrap{display:flex;flex-direction:column;align-items:center;gap:2px;}
.cbtn{width:52px;height:52px;border-radius:50%;border:none;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:transform 0.15s;}
.cbtn:hover{transform:scale(1.1);}
.cbtn:active{transform:scale(0.95);}
.cbtn-lbl{font-size:13px;color:#546e7a;}
.cbtn-green{background:#43a047;box-shadow:0 2px 8px rgba(67,160,71,0.45);}
.cbtn-blue{background:#1565c0;box-shadow:0 2px 8px rgba(21,101,192,0.4);}
.cbtn-gray{background:#546e7a;box-shadow:0 2px 8px rgba(84,110,122,0.4);}
.cbtn-gold{background:#f9a825;box-shadow:0 2px 8px rgba(249,168,37,0.4);}
.cbtn-red{background:#e53935;box-shadow:0 2px 8px rgba(229,57,53,0.4);}

/* RIGHT COLUMN */
.right-col{display:flex;flex-direction:column;gap:12px;overflow-y:auto;padding:20px 70px 20px 20px;justify-content:center;}
.divider{border-top:1px solid #c5d8f0;flex-shrink:0;}
.disease-section{flex:1;display:flex;flex-direction:column;min-height:0;}
.det-label{font-size:8px;color:#90a4ae;text-transform:uppercase;letter-spacing:0.8px;margin-bottom:4px;}
.disease-name{font-size:20px;font-weight:700;line-height:1.15;margin-bottom:3px;}
.disease-name.bcc{color:#111;}
.disease-name.nevus{color:#111;}
.disease-name.none{color:#111;}
.disease-conf{font-size:9px;color:#546e7a;margin-bottom:8px;}
.conf-val{color:#f9a825;font-weight:600;}
.disease-desc{font-size:14px;color:#546e7a;line-height:1.6;flex:1;overflow:hidden;}
.learn-wrap{margin-top:10px;flex-shrink:0;}
.learn-btn{font-size:9px;color:#1565c0;border:1px solid #1565c0;border-radius:6px;padding:5px 12px;display:inline-flex;align-items:center;gap:3px;}
.learn-btn:hover{background:#e3f2fd;}

/* SECONDARY SECTION */
.secondary{background:#dce8f5;border-top:1px solid #c5d8f0;padding:14px;}
.sec-title{font-size:15px;color:#1a3a8f;font-weight:700;text-transform:uppercase;letter-spacing:1px;margin-bottom:12px;}
.acc-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:12px;}
.acc-card{background:white;border-radius:10px;padding:12px;text-align:center;border:1px solid #c5d8f0;box-shadow:0 1px 5px rgba(26,58,143,0.06);}
.acc-num{font-size:22px;font-weight:700;}
.acc-lbl{font-size:9px;color:#546e7a;margin-top:3px;}
.acc-bcc{color:#c62828;}.acc-nevus{color:#1565c0;}.acc-all{color:#f9a825;}
.two-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px;}
.sec-card{background:white;border-radius:10px;padding:12px;border:1px solid #c5d8f0;box-shadow:0 1px 5px rgba(26,58,143,0.06);}
.sec-card-title{font-size:15px;font-weight:700;color:#1a3a8f;margin-bottom:10px;}
.tl-item{display:flex;align-items:center;gap:6px;margin-bottom:7px;}
.tl-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0;}
.tl-bar-wrap{flex:1;height:4px;border-radius:2px;overflow:hidden;}
.tl-bar{height:100%;border-radius:2px;transition:width 0.5s;}
.tl-lbl{font-size:13px;font-weight:600;min-width:54px;text-align:right;}
.tl-time{font-size:12px;color:#90a4ae;min-width:30px;text-align:right;}
.info-row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid #f0f4f8;}
.info-row:last-child{border-bottom:none;}
.info-key{font-size:13px;color:#546e7a;}
.info-val{font-size:14px;font-weight:700;}
.cm-table{width:auto;border-collapse:collapse;font-size:13px;margin:0 auto;}
.cm-table td{padding:5px 10px;text-align:center;border-radius:4px;}
.cm-head{font-weight:600;color:#546e7a;}
.cm-green{background:#e8f5e9;color:#2e7d32;font-weight:700;}
.cm-red{background:#fce4ec;color:#c62828;}
.cm-gray{background:#fafafa;color:#90a4ae;}
.tbl-wrap{background:white;border-radius:10px;border:1px solid #c5d8f0;overflow:hidden;margin-top:12px;}
.tbl-head{background:#1a3a8f;padding:9px 14px;font-size:15px;color:white;font-weight:600;letter-spacing:0.5px;}
table{width:100%;border-collapse:collapse;font-size:14px;}
thead tr{background:#EEF4FB;}
th{padding:8px 12px;text-align:left;color:#1a3a8f;font-weight:700;font-size:13px;}
td{padding:8px 12px;border-bottom:1px solid #f0f4f8;color:#37474f;}
tr:hover td{background:#f8fafd;}
.lbl-bcc{color:#111;font-weight:700;}.lbl-nevus{color:#111;font-weight:700;}.lbl-none{color:#333;}

/* LIMIT ALERT */
.overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(13,43,94,0.55);z-index:900;justify-content:center;align-items:flex-start;padding-top:80px;}
.overlay.show{display:flex;}
.alert-box{background:white;border-radius:16px;padding:28px 30px;max-width:380px;width:90%;text-align:center;box-shadow:0 8px 40px rgba(0,0,0,0.2);border-top:4px solid #f9a825;position:relative;}
.alert-close{position:absolute;top:12px;right:14px;background:none;border:none;font-size:20px;color:#90a4ae;cursor:pointer;line-height:1;}
.alert-close:hover{color:#1a3a8f;}
.alert-icon{font-size:36px;margin-bottom:10px;}
.alert-title{font-size:17px;font-weight:700;color:#0d2b5e;margin-bottom:6px;}
.alert-msg{font-size:13px;color:#546e7a;line-height:1.6;margin-bottom:18px;}
.alert-btn{background:#1a3a8f;color:white;border:none;padding:10px 30px;border-radius:8px;cursor:pointer;font-size:14px;font-weight:600;}
.alert-btn:hover{background:#2255cc;}

/* IMAGE MODAL */
.modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(13,43,94,0.7);z-index:950;justify-content:center;align-items:center;}
.modal.open{display:flex;}
.modal-box{background:white;border-radius:16px;padding:20px;max-width:440px;width:90%;box-shadow:0 8px 40px rgba(0,0,0,0.2);}
.modal-img{width:100%;border-radius:10px;margin-bottom:14px;max-height:280px;object-fit:contain;}
.modal-lbl{font-size:20px;font-weight:700;text-align:center;margin-bottom:4px;}
.modal-conf{font-size:13px;color:#546e7a;text-align:center;margin-bottom:4px;}
.modal-time{font-size:11px;color:#90a4ae;text-align:center;margin-bottom:16px;}
.modal-close-btn{width:100%;padding:10px;background:#1a3a8f;color:white;border:none;border-radius:8px;cursor:pointer;font-size:14px;font-weight:600;}
.modal-close-btn:hover{background:#2255cc;}

/* FOOTER */
.footer{background:#1a3a8f;padding:12px 20px;text-align:center;font-size:10px;color:rgba(255,255,255,0.55);}
.footer strong{color:white;}
</style>
</head>
<body>

<!-- HEADER -->
<div class="hdr">
  <div class="hdr-left">
    <h1>Acne &amp; Skin Disease Detection using Image Data</h1>
    <p>Government College University Faisalabad &nbsp;|&nbsp; FYP 2022&ndash;2026 &nbsp;|&nbsp; Supervisor: Dr. Usama Ahmed</p>
  </div>
 
</div>

<!-- MAIN GRID: fills screen height -->
<div class="main">

  <!-- LEFT: Camera + Images Panel -->
  <div class="panel">
    <div class="panel-title">Live Preview &amp; Captured Images</div>
    <div class="cam-grid">

      <!-- Row 1: slots 1 2 3 4 -->
      <div class="slot" id="slot0" onclick="openModal(0)"><span>1</span></div>
      <div class="slot" id="slot1" onclick="openModal(1)"><span>2</span></div>
      <div class="slot" id="slot2" onclick="openModal(2)"><span>3</span></div>
      <div class="slot" id="slot3" onclick="openModal(3)"><span>4</span></div>

      <!-- Row 2: slot5 | LIVE CAM (cols2-3, rows2-3) | slot6 -->
      <div class="slot" id="slot4" onclick="openModal(4)"><span>5</span></div>
      <div class="live-cam">
        <img id="camImg" src="/stream" alt="Live"
          onerror="this.style.display='none';document.getElementById('camPH').style.display='flex';">
        <div class="cam-placeholder" id="camPH" style="display:none;">
          <svg width="28" height="28" viewBox="0 0 24 24" fill="rgba(255,255,255,0.3)"><path d="M12 15.2A3.2 3.2 0 0 1 8.8 12 3.2 3.2 0 0 1 12 8.8 3.2 3.2 0 0 1 15.2 12 3.2 3.2 0 0 1 12 15.2M20 4h-3.17L15 2H9L7.17 4H4c-1.1 0-2 .9-2 2v12c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2z"/></svg>
          <span class="cam-lbl">OV2640 ESP32-CAM</span>
        </div>
        <div class="live-badge"><span class="live-dot"></span>LIVE</div>
        <div class="live-info">534ms inference</div>
      </div>
      <div class="slot" id="slot5" onclick="openModal(5)"><span>6</span></div>

      <!-- Row 3: slot7 | (cam) | slot8 -->
      <div class="slot" id="slot6" onclick="openModal(6)"><span>7</span></div>
      <!-- camera occupies cols2-3 row3 automatically -->
      <div class="slot" id="slot7" onclick="openModal(7)"><span>8</span></div>

      <!-- Row 4: slots 9 10 11 12 -->
      <div class="slot" id="slot8"  onclick="openModal(8)"><span>9</span></div>
      <div class="slot" id="slot9"  onclick="openModal(9)"><span>10</span></div>
      <div class="slot" id="slot10" onclick="openModal(10)"><span>11</span></div>
      <div class="slot" id="slot11" onclick="openModal(11)"><span>12</span></div>

    </div>
  </div>

  <!-- MIDDLE: Circle Buttons -->
  <div class="btn-col">
    <div class="cbtn-wrap" id="playWrap">
      <button class="cbtn cbtn-green" onclick="startDetection()" title="Start Detection">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="white"><path d="M8 5v14l11-7z"/></svg>
      </button>
      <span class="cbtn-lbl">Start</span>
    </div>
    <div class="cbtn-wrap" id="pauseWrap" style="display:none;">
      <button class="cbtn cbtn-red" onclick="pauseDetection()" title="Pause">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="white"><path d="M6 19h4V5H6v14zm8-14v14h4V5h-4z"/></svg>
      </button>
      <span class="cbtn-lbl">Pause</span>
    </div>
    <!-- Status pill below play/pause button -->
    

    <div class="cbtn-wrap" style="margin-top:4px;">
      <button class="cbtn cbtn-blue" onclick="captureOne()" title="Capture One">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="white"><path d="M12 15.2A3.2 3.2 0 0 1 8.8 12 3.2 3.2 0 0 1 12 8.8 3.2 3.2 0 0 1 15.2 12 3.2 3.2 0 0 1 12 15.2M20 4h-3.17L15 2H9L7.17 4H4c-1.1 0-2 .9-2 2v12c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V6c0-1.1-.9-2-2-2z"/></svg>
      </button>
      <span class="cbtn-lbl">Capture</span>
    </div>
    <div class="cbtn-wrap">
      <button class="cbtn cbtn-gray" onclick="clearAll()" title="Clear All">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="white"><path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z"/></svg>
      </button>
      <span class="cbtn-lbl">Clear</span>
    </div>
    <div class="cbtn-wrap">
      <button class="cbtn cbtn-gold" onclick="exportCSV()" title="Export CSV">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="#1a1a1a"><path d="M19 9h-4V3H9v6H5l7 7 7-7zM5 18v2h14v-2H5z"/></svg>
      </button>
      <span class="cbtn-lbl">Export</span>
    </div>
  </div>

  <!-- RIGHT: Pie + Disease Info -->
  <div class="panel right-col">

    <!-- Tiny Pie -->
    <canvas id="tinyPie" width="110" height="110" style="display:block;margin:0 auto;flex-shrink:0;"></canvas>
    <div style="width:100%;flex-shrink:0;">
      <div style="display:flex;align-items:center;gap:5px;margin-bottom:4px;">
        <div style="width:7px;height:7px;border-radius:50%;background:#ef5350;flex-shrink:0;"></div>
        <span style="font-size:9px;color:#546e7a;flex:1;">BCC</span>
        <span style="font-size:9px;font-weight:600;color:#1a2e4a;" id="pieBccPct">0%</span>
      </div>
      <div style="display:flex;align-items:center;gap:5px;margin-bottom:4px;">
        <div style="width:7px;height:7px;border-radius:50%;background:#1565c0;flex-shrink:0;"></div>
        <span style="font-size:9px;color:#546e7a;flex:1;">Nevus</span>
        <span style="font-size:9px;font-weight:600;color:#1a2e4a;" id="pieNevusPct">0%</span>
      </div>
      <div style="display:flex;align-items:center;gap:5px;">
        <div style="width:7px;height:7px;border-radius:50%;background:#b0bec5;flex-shrink:0;"></div>
        <span style="font-size:9px;color:#546e7a;flex:1;">None</span>
        <span style="font-size:9px;font-weight:600;color:#1a2e4a;" id="pieNonePct">0%</span>
      </div>
    </div>

    <div class="divider"></div>

    <!-- Disease Info -->
    <div class="disease-section">
      <div class="det-label">Primary Detection</div>
      <div class="disease-name none" id="diseaseName">No Detection</div>
      <div class="disease-conf" id="diseaseConf">Capture an image to begin</div>
      <div class="disease-desc" id="diseaseDesc">Point the camera at a skin lesion and press Start or Capture to begin detection.</div>
      <div class="learn-wrap">
        <div class="divider" style="margin-bottom:10px;"></div>
        <a class="learn-btn" id="learnLink" href="#" target="_blank" style="display:none;">
          <svg width="9" height="9" viewBox="0 0 24 24" fill="#1565c0"><path d="M19 19H5V5h7V3H5a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-7h-2v7zM14 3v2h3.59l-9.83 9.83 1.41 1.41L19 6.41V10h2V3h-7z"/></svg>
          Learn More
        </a>
      </div>
    </div>
  </div>

</div><!-- end main -->

<!-- SECONDARY / SCROLL -->
<div class="secondary">
  <div class="sec-title">Detailed Analysis</div>

 

  

  <!-- Model Performance + Training Dataset -->
  <div class="two-grid">
    <div class="sec-card">
      <div class="sec-card-title">Model Performance</div>
      <div class="info-row"><span class="info-key">Overall F1 Score</span><span class="info-val" style="color:#111;">83.9%</span></div>
      <div class="info-row"><span class="info-key">BCC F1 Score</span><span class="info-val" style="color:#111;">0.82</span></div>
      <div class="info-row"><span class="info-key">Nevus F1 Score</span><span class="info-val" style="color:#111;">0.86</span></div>
      <div class="info-row"><span class="info-key">Background F1</span><span class="info-val" style="color:#111;">1.00</span></div>
      <div class="info-row"><span class="info-key">DSP Time</span><span class="info-val" style="color:#111;">7 ms</span></div>
      <div class="info-row"><span class="info-key">Inference Time</span><span class="info-val" style="color:#111;">534 ms</span></div>
    </div>
    <div class="sec-card">
      <div class="sec-card-title">Training Dataset</div>
      <div class="info-row"><span class="info-key">Total Images</span><span class="info-val" style="color:#111;">2,800</span></div>
      <div class="info-row"><span class="info-key">BCC Images</span><span class="info-val" style="color:#111;">1,400</span></div>
      <div class="info-row"><span class="info-key">Nevus Images</span><span class="info-val" style="color:#111;">1,400</span></div>
      <div class="info-row"><span class="info-key">Input Size</span><span class="info-val" style="color:#111;">96x96 px</span></div>
      <div class="info-row"><span class="info-key">Architecture</span><span class="info-val" style="color:#111;">FOMO MobileNetV2</span></div>
      <div class="info-row"><span class="info-key">Training Epochs</span><span class="info-val" style="color:#111;">40</span></div>
    </div>
  </div>

  <!-- Confusion Matrix -->
  <div class="sec-card" style="margin-bottom:12px;">
    <div class="sec-card-title">Confusion Matrix &mdash; Validation Set</div>
    <div style="overflow-x:auto;">
      <table class="cm-table">
        <tr><td></td><td class="cm-head">Background</td><td class="cm-head">BCC</td><td class="cm-head">Nevus</td></tr>
        <tr><td class="cm-head">Background</td><td class="cm-green">100%</td><td class="cm-gray">0%</td><td class="cm-gray">0%</td></tr>
        <tr><td class="cm-head">BCC</td><td class="cm-red">20%</td><td class="cm-green">80%</td><td class="cm-gray">0%</td></tr>
        <tr><td class="cm-head">Nevus</td><td class="cm-red">15.4%</td><td class="cm-gray">0%</td><td class="cm-green">84.6%</td></tr>
      </table>
    </div>
  </div>

  <!-- Results Table -->
  <div class="tbl-wrap">
    <div class="tbl-head">All Captured Results</div>
    <div style="overflow-x:auto;">
      <table>
        <thead><tr><th>#</th><th>Result</th><th>Confidence</th><th>Time</th></tr></thead>
        <tbody id="resultsTable">
          <tr><td colspan="4" style="text-align:center;color:#90a4ae;padding:20px;">No captures yet.</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</div>

<!-- FOOTER -->
<div class="footer">
  <strong>Iqra Waheed (221982)</strong> &nbsp;|&nbsp;
  <strong>Rimsha Shahzadi (222027)</strong> &nbsp;|&nbsp;
  Supervisor: <strong>Dr. Usama Ahmed</strong> &nbsp;&mdash;&nbsp;
  Government College University Faisalabad &nbsp;|&nbsp; FYP 2022&ndash;2026
</div>

<!-- LIMIT ALERT: only one instance, cross to dismiss -->
<div class="overlay" id="limitOverlay">
  <div class="alert-box">
    <button class="alert-close" onclick="dismissAlert()" title="Close">&times;</button>
    <div class="alert-icon">&#9888;</div>
    <div class="alert-title">Image Limit Reached</div>
    <div class="alert-msg">
      You have captured <strong>12 images</strong> &mdash; the maximum allowed per session.<br>
      Press <strong>Clear All</strong> to start a new session.
    </div>
    <button class="alert-btn" onclick="dismissAlert()">OK, Got it</button>
  </div>
</div>

<!-- IMAGE MODAL -->
<div class="modal" id="imgModal" onclick="closeModal()">
  <div class="modal-box" onclick="event.stopPropagation()">
    <img id="modalImg" class="modal-img" src="" alt="">
    <div class="modal-lbl"  id="modalLbl"></div>
    <div class="modal-conf" id="modalConf"></div>
    <div class="modal-time" id="modalTime"></div>
    <button class="modal-close-btn" onclick="closeModal()">Close</button>
  </div>
</div>

<script>
// ── Disease Database ──────────────────────────────────────
const DISEASES = {
  bcc: {
    name: 'Basal Cell Carcinoma',
    cls:  'bcc',
    desc: 'Basal Cell Carcinoma is the most common form of skin cancer. It is caused by prolonged exposure to ultraviolet (UV) radiation from sunlight or tanning beds, which damages the DNA in skin cells. It typically appears as a pearly or waxy bump, a flat flesh-colored lesion, or a bleeding sore that heals and returns. Though it rarely spreads, early treatment prevents disfigurement and complications.',
    link: 'https://www.mayoclinic.org/diseases-conditions/basal-cell-carcinoma/symptoms-causes/syc-20354187'
  },
  nevus: {
    name: 'Nevus (Mole)',
    cls:  'nevus',
    desc: 'A Nevus is a common benign skin growth formed by clusters of pigment-producing melanocytes. Moles are generally harmless and appear brown or black. However, some can transform into melanoma — a dangerous skin cancer. Monitor moles using the ABCDE rule: Asymmetry, Border irregularity, Color variation, Diameter over 6mm, and Evolving shape or size. Consult a dermatologist if changes are noticed.',
    link: 'https://www.aad.org/public/diseases/a-z/moles-overview'
  },
  none: {
    name: 'No Disease Identified',
    cls:  'none',
    desc: 'No skin disease has been identified in the captured image. This may occur if the lesion is not clearly visible, the lighting is poor, or the camera is too far from the skin. For accurate detection, ensure the lesion fills most of the frame, with good lighting and the camera held 15-30cm from the skin surface.',
    link: null
  }
};

// ── State ────────────────────────────────────────────────
let isRunning    = false;
let lastCount    = 0;
let cachedData   = null;
let alertShown   = false; // show limit alert only once per session


// ── Camera Refresh ───────────────────────────────────────
function refreshCam() {
  const img = document.getElementById('camImg');
  img.src = '/stream?' + Date.now();
  setTimeout(refreshCam, 1000);
}
refreshCam();

// ── Poll results every 2s ────────────────────────────────
setInterval(fetchResults, 2000);

// ── Controls ─────────────────────────────────────────────
function startDetection() {
  fetch('/start').then(() => { isRunning = true; alertShown = false; updateHeaderUI(); });
}
function pauseDetection() {
  fetch('/pause').then(() => { isRunning = false; updateHeaderUI(); });
}
function captureOne() {
  fetch('/captureone').then(r => r.text()).then(t => {
    if (t === 'limit') {
      if (!alertShown) { showLimitAlert(); alertShown = true; }
      return;
    }
    setTimeout(fetchResults, 1500);
  });
}
function clearAll() {
  if (!confirm('Clear all 12 captured images and reset?')) return;
  fetch('/clear').then(() => {
    lastCount  = 0;
    trendData  = [];
    alertShown = false;
    for (let i = 0; i < 12; i++) resetSlot(i);
    dismissAlert();
    fetchResults();
    
  });
}
function exportCSV() { window.location.href = '/export'; }

// ── Header UI ────────────────────────────────────────────
function updateHeaderUI() {
  const pills  = [document.getElementById('statusPill')].filter(Boolean);
  const pw     = document.getElementById('playWrap');
  const paw    = document.getElementById('pauseWrap');
  pills.forEach(p => {
    if (!p) return;
    if (isRunning) { p.className='status-pill pill-run'; p.textContent='RUNNING'; }
    else           { p.className='status-pill pill-pause'; p.textContent='PAUSED'; }
  });
  if (isRunning) { pw.style.display='none'; paw.style.display='flex'; }
  else           { pw.style.display='flex'; paw.style.display='none'; }
}

// ── Slots ─────────────────────────────────────────────────
function resetSlot(i) {
  const s = document.getElementById('slot' + i);
  if (!s) return;
  s.className = 'slot';
  s.innerHTML = '<span>' + (i+1) + '</span>';
  s.onclick = () => openModal(i);
}
function fillSlot(i, cap) {
  const s = document.getElementById('slot' + i);
  if (!s) return;
  const cls = cap.hasDetection ? cap.label : 'none';
  const lbl = cap.hasDetection ? cap.label.toUpperCase() : 'NONE';
  const conf= cap.hasDetection ? (cap.confidence*100).toFixed(0)+'%' : '';

  
  s.className = 'slot filled ' + cls;
  s.innerHTML = `<img src="/image?id=${i}&t=${Date.now()}" alt="Cap${i+1}" loading="lazy">
    <div class="slot-badge ${cls}">${lbl} ${conf}</div>`;
  s.onclick = () => openModal(i);
}

// ── Modal ─────────────────────────────────────────────────
function openModal(idx) {
  if (!cachedData || !cachedData.captures[idx]) return;
  const c    = cachedData.captures[idx];
  const cls  = c.hasDetection ? c.label : 'none';
  const lbl  = c.hasDetection ? c.label.toUpperCase() : 'NO DETECTION';
  const conf = c.hasDetection ? (c.confidence*100).toFixed(1)+'%' : '-';
  document.getElementById('modalImg').src = '/image?id='+idx+'&t='+Date.now();
  document.getElementById('modalLbl').textContent  = lbl;
  document.getElementById('modalLbl').className    = 'modal-lbl lbl-'+cls;
  document.getElementById('modalConf').textContent = 'Confidence: '+conf;
  document.getElementById('modalTime').textContent = 'Captured at: '+c.timestamp;
  document.getElementById('imgModal').classList.add('open');
}
function closeModal() { document.getElementById('imgModal').classList.remove('open'); }

// ── Limit Alert ───────────────────────────────────────────
function showLimitAlert()  { document.getElementById('limitOverlay').classList.add('show'); }
function dismissAlert()    { document.getElementById('limitOverlay').classList.remove('show'); }

// ── Pie Chart ─────────────────────────────────────────────
function drawPie(bcc, nevus, none) {
  const c = document.getElementById('tinyPie');
  const ctx = c.getContext('2d');
  const cx=55, cy=55, r=48;
  ctx.clearRect(0,0,110,110);
  const total = bcc+nevus+none;
  if (!total) {
    ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI);
    ctx.fillStyle='#EEF4FB'; ctx.fill();
    ctx.fillStyle='#111';
    ctx.fillText('No data',cx,cy+4);
    return;
  }
  const cols = [['#ef5350',bcc],['#1565c0',nevus],['#b0bec5',none]];
  let s = -Math.PI/2;
  cols.forEach(([col,v]) => {
    if (!v) return;
    const sl=(v/total)*2*Math.PI;
    ctx.beginPath(); ctx.moveTo(cx,cy); ctx.arc(cx,cy,r,s,s+sl);
    ctx.closePath(); ctx.fillStyle=col; ctx.fill();
    ctx.strokeStyle='white'; ctx.lineWidth=2; ctx.stroke();
    const mid=s+sl/2;
    ctx.fillStyle='white'; ctx.font='bold 10px Segoe UI'; ctx.textAlign='center';
    ctx.fillText(Math.round(v/total*100)+'%', cx+30*Math.cos(mid), cy+30*Math.sin(mid)+3);
    s+=sl;
  });
  // donut hole
  ctx.beginPath(); ctx.arc(cx,cy,20,0,2*Math.PI);
  ctx.fillStyle='white'; ctx.fill();
  ctx.fillStyle='#1a3a8f'; ctx.font='bold 12px Segoe UI'; ctx.textAlign='center';
  ctx.fillText(total, cx, cy+4);
}





// ── Disease Card ──────────────────────────────────────────
function updateDiseaseCard(bcc, nevus, bccAvg, nevusAvg) {
  let key = 'none';
   if (bcc>0 || nevus>0) {
    let bccScore   = bcc * ((bccAvg   || 0) / 100);
    let nevusScore = nevus * ((nevusAvg || 0) / 100);
    key = bccScore >= nevusScore ? 'bcc' : 'nevus';
  }
  const D = DISEASES[key];
  const nameEl = document.getElementById('diseaseName');
  const confEl = document.getElementById('diseaseConf');
  const descEl = document.getElementById('diseaseDesc');
  const lnkEl  = document.getElementById('learnLink');

  nameEl.textContent = D.name;
  nameEl.className   = 'disease-name ' + D.cls;
  descEl.textContent = D.desc;

  const avg = key==='bcc' ? bccAvg : key==='nevus' ? nevusAvg : -1;
  const cnt = key==='bcc' ? bcc    : key==='nevus' ? nevus    : 0;
  confEl.innerHTML = avg>=0
    ? `Avg: <span class="conf-val">${avg.toFixed(1)}%</span> &nbsp;&middot;&nbsp; ${cnt} image${cnt!==1?'s':''}`
    : 'Capture an image to begin';

  if (D.link) { lnkEl.href=D.link; lnkEl.style.display='inline-flex'; }
  else        { lnkEl.style.display='none'; }
}

// ── Main Fetch ────────────────────────────────────────────
function fetchResults() {
  fetch('/results')
    .then(r => r.json())
    .then(data => {
      cachedData = data;
      isRunning  = data.running;
      updateHeaderUI();

      const bcc=data.bccCount, nevus=data.nevusCount, none=data.noneCount, total=data.total;

      // Limit alert — show only once
      if (data.limitReached && !alertShown) { showLimitAlert(); alertShown=true; }

      // Pie chart
      drawPie(bcc, nevus, none);
      const tot = bcc+nevus+none || 1;
      document.getElementById('pieBccPct').textContent   = Math.round(bcc/tot*100)+'%';
      document.getElementById('pieNevusPct').textContent = Math.round(nevus/tot*100)+'%';
      document.getElementById('pieNonePct').textContent  = Math.round(none/tot*100)+'%';

      // Disease card
      updateDiseaseCard(bcc, nevus, data.bccAvg, data.nevusAvg);

     
      // Image slots
      if (data.captures) {
        if (total===0 && lastCount>0) { for(let i=0;i<12;i++) resetSlot(i); lastCount=0; trendData=[]; }
        for (let i=lastCount; i<data.captures.length; i++) {
  
 
  fillSlot(i, data.captures[i]);
}
lastCount = data.captures.length;

     
      }

    

      // Results table
     
      const tbody = document.getElementById('resultsTable');
      if (!data.captures || data.captures.length===0) {
       tbody.innerHTML='<tr><td colspan="4" style="text-align:center;color:#90a4ae;padding:20px;">No captures yet.</td></tr>';
        return;
}
      let html='';
      for(let i=data.captures.length-1; i>=0; i--) {
        const c    = data.captures[i];
        const lbl  = c.hasDetection
          ? `<span class="lbl-${c.label}">${c.label.toUpperCase()}</span>`
          : `<span class="lbl-none">No Detection</span>`;
        const conf = c.hasDetection ? (c.confidence*100).toFixed(1)+'%' : '-';
        html+=`<tr><td>${i+1}</td><td>${lbl}</td><td>${conf}</td><td>${c.timestamp}</td></tr>`;
      }
      tbody.innerHTML=html;
    })
    .catch(e => console.log('Fetch error:',e));
}


// Init
drawPie(0,0,0);

</script>
</body>
</html>
)HTML";

void handleRoot(){server.send_P(200,"text/html",htmlPage);}
void handleStart(){isRunning=true;limitAlert=false;server.send(200,"text/plain","ok");Serial.println("STARTED");}
void handlePause(){isRunning=false;server.send(200,"text/plain","ok");Serial.println("PAUSED");}
void handleCaptureOne(){if(captureCount>=MAX_CAPTURES){limitAlert=true;server.send(200,"text/plain","limit");return;}doCapture=true;server.send(200,"text/plain","ok");}
void handleClear(){for(int i=0;i<captureCount;i++){if(captures[i].jpegData){free(captures[i].jpegData);captures[i].jpegData=nullptr;}}captureCount=0;limitAlert=false;isRunning=false;for(int i=0;i<MAX_CAPTURES;i++)captures[i]={"",0,false,"",nullptr,0};server.send(200,"text/plain","cleared");Serial.println("Cleared");}
void handleStream(){camera_fb_t*fb=esp_camera_fb_get();if(!fb){server.send(500,"text/plain","err");return;}server.sendHeader("Cache-Control","no-cache,no-store");server.send_P(200,"image/jpeg",(const char*)fb->buf,fb->len);esp_camera_fb_return(fb);}
void handleImage(){if(!server.hasArg("id")){server.send(400,"text/plain","Missing id");return;}int id=server.arg("id").toInt();if(id<0||id>=captureCount||!captures[id].jpegData){server.send(404,"text/plain","Not found");return;}server.sendHeader("Cache-Control","no-cache");server.send_P(200,"image/jpeg",(const char*)captures[id].jpegData,captures[id].jpegLen);}

void handleResults(){
  int bccC=0,nevC=0,noneC=0;float bccT=0,nevT=0,allT=0;int bccD=0,nevD=0,allD=0;
  String json="{";
  json+="\"running\":"+String(isRunning?"true":"false")+",";
  json+="\"limitReached\":"+String(limitAlert?"true":"false")+",";
  json+="\"total\":"+String(captureCount)+",";
  json+="\"captures\":[";
  for(int i=0;i<captureCount;i++){
    CaptureResult&c=captures[i];
    json+="{\"label\":\""+c.label+"\",\"confidence\":"+String(c.confidence,4)+",\"hasDetection\":"+String(c.hasDetection?"true":"false")+",\"timestamp\":\""+c.timestamp+"\"}";
    if(i<captureCount-1)json+=",";
    if(c.hasDetection){allT+=c.confidence;allD++;if(c.label=="bcc"){bccT+=c.confidence;bccD++;bccC++;}if(c.label=="nevus"){nevT+=c.confidence;nevD++;nevC++;}}
    else if(c.label!="")noneC++;
  }
  json+="],\"bccCount\":"+String(bccC)+",\"nevusCount\":"+String(nevC)+",\"noneCount\":"+String(noneC)+",\"detectedCount\":"+String(allD)+",";
  float avgAcc=allD>0?(allT/allD)*100.0:-1,bccAvg=bccD>0?(bccT/bccD)*100.0:-1,nevAvg=nevD>0?(nevT/nevD)*100.0:-1;
  json+="\"avgAcc\":"+String(avgAcc,2)+",\"bccAvg\":"+String(bccAvg,2)+",\"nevusAvg\":"+String(nevAvg,2)+"}";
  server.sendHeader("Access-Control-Allow-Origin","*");server.send(200,"application/json",json);
}

void handleExport(){
  String csv="No,Label,Confidence (%),Time\n";
  for(int i=0;i<captureCount;i++){CaptureResult&c=captures[i];csv+=String(i+1)+","+(c.hasDetection?c.label:"No Detection")+","+String(c.hasDetection?c.confidence*100.0:0.0,2)+","+c.timestamp+"\n";}
  float t=0;int d=0;for(int i=0;i<captureCount;i++)if(captures[i].hasDetection){t+=captures[i].confidence;d++;}
  csv+="\nTotal Captures,"+String(captureCount)+"\n";if(d>0)csv+="Average Accuracy,"+String((t/d)*100.0,2)+"%\n";
  server.sendHeader("Content-Disposition","attachment; filename=dermscan_results.csv");server.send(200,"text/csv",csv);
}

void runInference(){
  if(captureCount>=MAX_CAPTURES){limitAlert=true;isRunning=false;Serial.println("Limit: 12 images. Clear to continue.");return;}
  camera_fb_t*fb=esp_camera_fb_get();if(!fb){Serial.println("JPEG capture failed");return;}
  uint8_t*jpegCopy=(uint8_t*)ps_malloc(fb->len);size_t jpegLen=0;
  if(jpegCopy){memcpy(jpegCopy,fb->buf,fb->len);jpegLen=fb->len;}
  esp_camera_fb_return(fb);
  snapshot_buf=(uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS*EI_CAMERA_RAW_FRAME_BUFFER_ROWS*EI_CAMERA_FRAME_BYTE_SIZE);
  if(!snapshot_buf){if(jpegCopy)free(jpegCopy);return;}
  ei::signal_t signal;signal.total_length=EI_CLASSIFIER_INPUT_WIDTH*EI_CLASSIFIER_INPUT_HEIGHT;signal.get_data=&ei_camera_get_data;
  if(!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH,EI_CLASSIFIER_INPUT_HEIGHT,snapshot_buf)){free(snapshot_buf);if(jpegCopy)free(jpegCopy);return;}
  ei_impulse_result_t result={0};EI_IMPULSE_ERROR err=run_classifier(&signal,&result,debug_nn);free(snapshot_buf);
  if(err!=EI_IMPULSE_OK){Serial.printf("Classifier error: %d\n",err);if(jpegCopy)free(jpegCopy);return;}
  CaptureResult&cap=captures[captureCount];cap.hasDetection=false;cap.label="";cap.confidence=0;cap.jpegData=jpegCopy;cap.jpegLen=jpegLen;
  struct tm timeinfo;
if(getLocalTime(&timeinfo)){
  char buf[20];
  strftime(buf, sizeof(buf), "%I:%M:%S %p", &timeinfo);
  cap.timestamp = String(buf);
} else {
  cap.timestamp = String(millis()) + "ms";
}
  Serial.printf("Predictions (DSP:%dms Class:%dms):\n",result.timing.dsp,result.timing.classification);
#if EI_CLASSIFIER_OBJECT_DETECTION==1
  float bestConf=0;String bestLabel="";
  for(uint32_t i=0;i<result.bounding_boxes_count;i++){ei_impulse_result_bounding_box_t bb=result.bounding_boxes[i];if(bb.value<0.3)continue;Serial.printf("  %s (%.1f%%)\n",bb.label,bb.value*100);if(bb.value>bestConf){bestConf=bb.value;bestLabel=String(bb.label);}}
  if(bestConf > 0.3){
    cap.hasDetection = true;
    cap.label = bestLabel;
    cap.confidence = bestConf;
    Serial.printf(">> %s (%.1f%%)\n", bestLabel.c_str(), bestConf*100);
} else {
    Serial.println(">> No confident detection. Raw scores:");
    for(uint32_t i=0; i<result.bounding_boxes_count; i++){
        Serial.printf("   [%s] = %.3f\n", 
            result.bounding_boxes[i].label, 
            result.bounding_boxes[i].value);
    }
    cap.label = "none";
}
#endif
  captureCount++;
}

void setup(){
  Serial.begin(115200);Serial.println("\n=== Acne & Skin Disease Detection FYP V5 Final ===");
  if(!ei_camera_init())Serial.println("Camera FAILED!");else Serial.println("Camera OK");
  for(int i=0;i<MAX_CAPTURES;i++)captures[i]={"",0,false,"",nullptr,0};
  Serial.printf("Connecting to WiFi: %s\n",ssid);
  WiFi.begin(ssid,password);int tries=0;
  while(WiFi.status()!=WL_CONNECTED&&tries<40){delay(500);Serial.print(".");tries++;}
  
  if(WiFi.status()==WL_CONNECTED){
    Serial.println("\nWiFi Connected!");
    Serial.print("Open: http://");
    Serial.println(WiFi.localIP());

    // ── NTP Time Sync (Pakistan = UTC+5) ──
    configTime(18000, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Syncing time");
    struct tm timeinfo;
    int ntpTries = 0;
    while(!getLocalTime(&timeinfo) && ntpTries < 10){
      delay(500);
      Serial.print(".");
      ntpTries++;
    }
    if(getLocalTime(&timeinfo)){
      Serial.println("\nTime synced OK!");
    } else {
      Serial.println("\nTime sync FAILED — will use ms fallback");
    }
    // ──────────────────────────────────────

  } else {
    Serial.println("\nWiFi FAILED!");
  }

  server.on("/",handleRoot);server.on("/start",handleStart);server.on("/pause",handlePause);
  server.on("/captureone",handleCaptureOne);server.on("/clear",handleClear);
  server.on("/stream",handleStream);server.on("/image",handleImage);
  server.on("/results",handleResults);server.on("/export",handleExport);
  server.begin();Serial.println("Web server started!");
}

void loop(){
  server.handleClient();
  if(doCapture){doCapture=false;runInference();return;}
  if(isRunning){runInference();delay(500);}else delay(100);
}

bool ei_camera_init(void){
  if(is_initialised)return true;
  esp_err_t err=esp_camera_init(&camera_config);if(err!=ESP_OK){Serial.printf("Camera init failed: 0x%x\n",err);return false;}
  sensor_t*s=esp_camera_sensor_get();if(s->id.PID==OV3660_PID){s->set_vflip(s,1);s->set_brightness(s,1);s->set_saturation(s,0);}
  is_initialised=true;return true;
}
void ei_camera_deinit(void){esp_err_t err=esp_camera_deinit();if(err!=ESP_OK){Serial.println("Camera deinit failed");return;}is_initialised=false;}
bool ei_camera_capture(uint32_t img_width,uint32_t img_height,uint8_t*out_buf){
  if(!is_initialised){Serial.println("Camera not initialized");return false;}
  camera_fb_t*fb=esp_camera_fb_get();if(!fb){Serial.println("Capture failed");return false;}
  bool converted=fmt2rgb888(fb->buf,fb->len,PIXFORMAT_JPEG,snapshot_buf);esp_camera_fb_return(fb);
  if(!converted){Serial.println("Conversion failed");return false;}
  if((img_width!=EI_CAMERA_RAW_FRAME_BUFFER_COLS)||(img_height!=EI_CAMERA_RAW_FRAME_BUFFER_ROWS)){ei::image::processing::crop_and_interpolate_rgb888(out_buf,EI_CAMERA_RAW_FRAME_BUFFER_COLS,EI_CAMERA_RAW_FRAME_BUFFER_ROWS,out_buf,img_width,img_height);}
  return true;
}
static int ei_camera_get_data(size_t offset,size_t length,float*out_ptr){
  size_t pixel_ix=offset*3,pixels_left=length,out_ptr_ix=0;
  while(pixels_left!=0){out_ptr[out_ptr_ix]=(snapshot_buf[pixel_ix+2]<<16)+(snapshot_buf[pixel_ix+1]<<8)+snapshot_buf[pixel_ix];out_ptr_ix++;pixel_ix+=3;pixels_left--;}
  return 0;
}
#if !defined(EI_CLASSIFIER_SENSOR)||EI_CLASSIFIER_SENSOR!=EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif

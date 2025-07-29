#include "web_templates.h"

// Full template for main UI
const char MAIN_PAGE_TEMPLATE_1[] = R"rawliteral(
<html>
<head>
<style>
body { font-family: monospace; background: #f8f9fa; margin: 0; padding: 0; }
pre { font-family: monospace; font-size: 16px; text-align: center; margin: 0 0 12px 0; }
#container { display: flex; flex-direction: row; justify-content: center; align-items: flex-start; }
#main-settings { flex: 1; max-width: 420px; margin-right: 32px; }
#slider-container { width: 80px; display: flex; flex-direction: column; align-items: center; }
.vertical-slider { writing-mode: bt-lr; -webkit-appearance: slider-vertical; width: 40px; height: 300px; margin-top: 24px; }
.advanced-section { background: #e9ecef; border-radius: 8px; margin-top: 18px; padding: 10px 14px; }
.adv-toggle { cursor: pointer; color: #007bff; text-decoration: underline; margin-bottom: 8px; display: inline-block; }
#disconnected { color: #b00; font-weight: bold; margin-top: 10px; }
.form-row { display: flex; align-items: center; margin-bottom: 8px; }
.form-row label { margin-right: 8px; }
</style>
</head>
<body>
<pre>{{ASCII_ART}}</pre>
<h2 style='text-align:center;margin:0 0 18px 0;'>SleepyLora Blinds Config</h2>
<div style='text-align:center;margin-bottom:18px;'>
  <a href="/help" style="font-size:1.1em;color:#007bff;text-decoration:underline;">Help</a>
</div>
<div id="disconnected" style="display:none; text-align:center; color:#b00; font-weight:bold; margin-bottom:10px;">Device sleeping or disconnected</div>
<div id='livepos' style='text-align:center;font-size:1.1em;margin-bottom:16px;'>
Raw Position: <span id='rawpos'>{{RAW_POS}}</span>  Blind Position: <span id='blindpos'>{{BLIND_POS}}0</span>%<br>
Status: <span id='blindstatus'>{{BLIND_STATUS}}</span> | Calibration: <span id='calibstate'>{{CALIBRATION_STATE}}</span>
</div>
<div id="container">
  <div id="main-settings">
    <div style="margin-top:10px;">
      <div style="font-weight:bold; margin-bottom:2px;">Closed Limit</div>
      <form method='POST' action='/set_closed' style='display: flex; align-items: center; gap: 8px; margin-bottom:4px;'>
        <input type='number' name='closed_limit' id='closed_limit' value='{{CLOSED_LIMIT}}' style='width:100px;'>
        <button type='submit' name='manual_closed' value='1'>Set Manually</button>
        <button type='submit'>Set Current Position</button>
      </form>
    </div>
    <div style="margin-top:10px;">
      <div style="font-weight:bold; margin-bottom:2px;">Open Limit</div>
      <form method='POST' action='/set_open' style='display: flex; align-items: center; gap: 8px; margin-bottom:4px;'>
        <input type='number' name='open_limit' id='open_limit' value='{{OPEN_LIMIT}}' style='width:100px;'>
        <button type='submit' name='manual_open' value='1'>Set Manually</button>
        <button type='submit'>Set Current Position</button>
      </form>
    </div>
    <form method='POST' action='/set_move_time' style='margin-top:10px;'>
      <input type='number' name='move_time' min='0' max='60000' value='{{MOVE_TIME}}'> Move Time (ms)
      <button type='submit'>Set</button>
    </form>
    <div class="adv-toggle" onclick="toggleAdvanced()">Show Advanced Settings &#9660;</div>
    <div id="advanced" class="advanced-section" style="display:none;">
      <form method='POST' action='/calibrate' style='margin-top:10px;'>
        <button type='submit'>Auto Calibrate</button>
      </form>
      <form method='POST' action='/set_engage_time' style='margin-top:10px; display: flex; align-items: center;'>
        <label for='engage_time' style='margin-right:8px; min-width:120px;'>Engage Time (ms):</label>
        <input type='number' name='engage_time' id='engage_time' min='0' max='10000' value='{{ENGAGE_TIME}}' style='width:100px; margin-right:8px;'>
        <button type='submit'>Set</button>
      </form>
      <form method='POST' action='/set_disengage_time' style='margin-top:10px; display: flex; align-items: center;'>
        <label for='disengage_time' style='margin-right:8px; min-width:120px;'>Disengage Time (ms):</label>
        <input type='number' name='disengage_time' id='disengage_time' min='0' max='10000' value='{{DISENGAGE_TIME}}' style='width:100px; margin-right:8px;'>
        <button type='submit'>Set</button>
      </form>
      <div style="margin-top:18px;">
        <div class="adv-toggle" onclick="togglePID()">PID Controller &#9660;</div>
        <div id="pid-settings" style="display:none; padding-left:10px;">
          <form method='POST' action='/set_pid' style='margin-top:8px;'>
            <div class="form-row"><label for='Kp'>Kp:</label><input type='number' step='0.01' name='Kp' id='Kp' value='{{PID_KP}}'></div>
            <div class="form-row"><label for='Ki'>Ki:</label><input type='number' step='0.0001' name='Ki' id='Ki' value='{{PID_KI}}'></div>
            <div class="form-row"><label for='Kd'>Kd:</label><input type='number' step='0.0001' name='Kd' id='Kd' value='{{PID_KD}}'></div>
            <div class="form-row"><label for='Deadband'>Deadband:</label><input type='number' step='0.01' name='Deadband' id='Deadband' value='{{PID_DEADBAND}}'></div>
            <div class="form-row"><label for='IntegralMin'>Integral Min:</label><input type='number' step='0.01' name='IntegralMin' id='IntegralMin' value='{{PID_IMIN}}'></div>
            <div class="form-row"><label for='IntegralMax'>Integral Max:</label><input type='number' step='0.01' name='IntegralMax' id='IntegralMax' value='{{PID_IMAX}}'></div>
            <div class="form-row"><label for='DerivMin'>Deriv Min:</label><input type='number' step='0.01' name='DerivMin' id='DerivMin' value='{{PID_DMIN}}'></div>
            <div class="form-row"><label for='DerivMax'>Deriv Max:</label><input type='number' step='0.01' name='DerivMax' id='DerivMax' value='{{PID_DMAX}}'></div>
            <div class="form-row"><label for='DerivAlpha'>Deriv Alpha:</label><input type='number' step='0.001' name='DerivAlpha' id='DerivAlpha' value='{{PID_DALPHA}}'></div>
            <div class="form-row"><label for='PwmAlpha'>PWM Alpha:</label><input type='number' step='0.001' name='PwmAlpha' id='PwmAlpha' value='{{PID_PALPHA}}'></div>
            <button type='submit'>Save PID</button>
          </form>
        </div>
      </div>
      <div style="margin-top:18px;">
        <div class="adv-toggle" onclick="toggleStall()">Stall & Engage Detection &#9660;</div>
        <div id="stall-settings" style="display:none; padding-left:10px;">
          <form method='POST' action='/set_stall' style='margin-top:8px;'>
            <div class="form-row"><label for='EngageThresh'>Engage Threshold:</label><input type='number' step='1' name='EngageThresh' id='EngageThresh' value='{{ENGAGE_THRESH}}'></div>
            <div class="form-row"><label for='EngageSampleCount'>Engage Sample Count:</label><input type='number' min='2' max='20' name='EngageSampleCount' id='EngageSampleCount' value='{{ENGAGE_SAMPLE_COUNT}}'></div>
            <div class="form-row"><label for='StallThresh'>Stall Threshold:</label><input type='number' step='1' name='StallThresh' id='StallThresh' value='{{STALL_THRESH}}'></div>
            <div class="form-row"><label for='StallSamples'>Stall Sample Count:</label><input type='number' min='2' max='20' name='StallSamples' id='StallSamples' value='{{STALL_SAMPLES}}'></div>
            <button type='submit'>Save Detection</button>
          </form>
        </div>
      </div>
      <div style="margin-top:18px;">
        <div class="adv-toggle" onclick="togglePWM()">Motor PWM Settings &#9660;</div>
        <div id="pwm-settings" style="display:none; padding-left:10px;">
          <form method='POST' action='/set_all_pwm' style='margin-top:10px;'>
            <div class="form-row">
              <label for='engage_pwm'>Engage PWM:</label>
              <input type='number' name='engage_pwm' min='0' max='255' value='{{ENGAGE_PWM}}'>
            </div>
            <div class="form-row">
              <label for='disengage_pwm'>Disengage PWM:</label>
              <input type='number' name='disengage_pwm' min='0' max='255' value='{{DISENGAGE_PWM}}'>
            </div>
            <div class="form-row">
              <label for='pwm_min'>Min PWM:</label>
              <input type='number' name='pwm_min' min='0' max='255' value='{{PWM_MIN}}'>
            </div>
            <div class="form-row">
              <label for='pwm_max'>Max PWM:</label>
              <input type='number' name='pwm_max' min='0' max='255' value='{{PWM_MAX}}'>
            </div>
            <div class="form-row">
              <label for='pwm_freq'>PWM Frequency (Hz):</label>
              <input type='number' name='pwm_freq' min='100' max='40000' value='{{PWM_FREQ}}'>
            </div>
            <button type='submit'>Set All PWM</button>
          </form>
        </div>
      </div>
    </div>
)rawliteral";
#ifdef DEVICE_ROLE_MASTER
const char DEVICE_SPECIFIC_PAGE_TEMPLATE[] = R"rawliteral(
    <!-- Slave Devices Section -->
    <div class="advanced-section" style="margin-top:18px;">
      <div class="adv-toggle" onclick="toggleSlaves()">Slave Devices &#9660;</div>
      <div id="slaves-section" style="display:none; padding-left:10px;">
        <form method="POST" action="/scan_slaves" style="margin-bottom:12px;">
          <button type="submit">Scan for Slaves</button>
        </form>
        <div id="slave-list">
          {{SLAVE_LIST}}
        </div>
      </div>
    </div>
    <!-- End Slave Devices Section -->
    <div class="advanced-section" style="margin-top:18px;">
      <div class="adv-toggle" onclick="toggleKeys()">Device Keys & Gateway Configuration &#9660;</div>
      <div id="keys-section" style="display:none; padding-left:10px;">
        <form method='POST' action='/set_keys' style='margin-bottom:10px;'>
          <div class="form-row"><label for='gateway_id'>Gateway ID (hex):</label><input type='text' name='gateway_id' id='gateway_id' value='{{GATEWAY_ID}}' maxlength='8' style='width:120px;' pattern="[0-9a-fA-F]{1,8}" title="8 hex digits" oninput="this.value=this.value.replace(/[^0-9a-fA-F]/g,'').slice(0,8)"></div>
          <div class="form-row"><label for='aes_key'>AES Key (hex):</label><input type='text' name='aes_key' id='aes_key' value='{{AES_KEY}}' maxlength='32' style='width:180px;' pattern="[0-9a-fA-F]{32}" title="32 hex digits" oninput="this.value=this.value.replace(/[^0-9a-fA-F]/g,'').slice(0,32)"></div>
          <div class="form-row"><label for='hmac_key'>HMAC Key (hex):</label><input type='text' name='hmac_key' id='hmac_key' value='{{HMAC_KEY}}' maxlength='20' style='width:120px;' pattern="[0-9a-fA-F]{20}" title="20 hex digits" oninput="this.value=this.value.replace(/[^0-9a-fA-F]/g,'').slice(0,20)"></div>
          <button type='submit' style='margin-left:12px;'>Set Keys</button>
        </form>
        <form method='POST' action='/loadfile' enctype='multipart/form-data'>
          <input type='file' name='config'>
          <button type='submit'>Load Keys/Config (JSON)</button>
        </form>
        <div style="font-size:0.95em; color:#666;">You can set keys and gateway manually above, or load them from a JSON file exported from another device.</div>
      </div>
    </div>
)rawliteral";
#else
const char DEVICE_SPECIFIC_PAGE_TEMPLATE[] = R"rawliteral(
    <!-- Slave Device Section -->
    <div class="advanced-section" style="margin-top:18px;">
      <div class="adv-toggle" onclick="toggleSlaveNumber()">Slave Number &#9660;</div>
      <div id="slave-number-section" style="display:none; padding-left:10px;">
        <form method="POST" action="/set_slave_number" style="margin-bottom:10px;">
          <label for="slave_number" style="font-weight:bold;">Slave Number:</label><br>
          <select name="slave_number" id="slave_number" style="width:100%;padding:6px;margin-bottom:10px;">
            <option value="1" {{SLAVE_NUM_1}}>1</option>
            <option value="2" {{SLAVE_NUM_2}}>2</option>
            <option value="3" {{SLAVE_NUM_3}}>3</option>
            <option value="4" {{SLAVE_NUM_4}}>4</option>
            <option value="5" {{SLAVE_NUM_5}}>5</option>
            <option value="6" {{SLAVE_NUM_6}}>6</option>
            <option value="7" {{SLAVE_NUM_7}}>7</option>
            <option value="8" {{SLAVE_NUM_8}}>8</option>
            <option value="9" {{SLAVE_NUM_9}}>9</option>
            <option value="10" {{SLAVE_NUM_10}}>10</option>
            <option value="11" {{SLAVE_NUM_11}}>11</option>
            <option value="12" {{SLAVE_NUM_12}}>12</option>
          </select><br>
          <button type='submit'>Save</button>
        </form>
      </div>
    </div>
)rawliteral";
#endif
const char MAIN_PAGE_TEMPLATE_2[] = R"rawliteral(
    <!-- OTA and Keys Section -->
    <div class="advanced-section" style="margin-top:18px;">
      <div class="adv-toggle" onclick="toggleOTA()">Firmware Update (OTA) &#9660;</div>
      <div id="ota-section" style="display:none; padding-left:10px;">
        <form method='POST' action='/update' enctype='multipart/form-data' style='margin-bottom:10px;'>
          <input type='file' name='update'>
          <button type='submit'>Update Firmware</button>
        </form>
        <div style="font-size:0.95em; color:#666; margin-bottom:8px;">Upload a .bin file to update device firmware.</div>
      </div>
    </div>

    <div class="advanced-section" style="margin-top:18px;">
      <div class="adv-toggle" onclick="toggleWiFi()">WiFi Settings &#9660;</div>
      <div id="wifi-section" style="display:none; padding-left:10px;">
        <form method='POST' action='/set_wifi' style='margin-bottom:10px;'>
          <div class="form-row"><label for='wifi_ssid'>SSID:</label><input type='text' name='wifi_ssid' id='wifi_ssid' value='{{WIFI_SSID}}' maxlength='32' style='width:180px;'></div>
          <div class="form-row"><label for='wifi_pass'>Password:</label><input type='password' name='wifi_pass' id='wifi_pass' value='{{WIFI_PASS}}' maxlength='64' style='width:180px;'></div>
          <div class="form-row"><input type='checkbox' name='wifi_enable' id='wifi_enable' {{WIFI_ENABLE}}> <label for='wifi_enable'>Connect to WiFi (otherwise AP mode)</label></div>
          <button type='submit' style='margin-left:12px;'>Save WiFi Settings</button>
        </form>
        <div id="wifi-status" style="font-size:0.95em; color:#666;">{{WIFI_STATUS}}</div>
      </div>
    </div>
  </div>
  <div id="slider-container">
    <input type="range" min="0" max="100" value="{{BLIND_POS}}" class="vertical-slider" id="blindSlider" orient="vertical">
    <div style="margin-top:12px;">Blind<br>Position</div>
  </div>
</div>
<form action="/close" method="POST" style="margin-top:32px;text-align:center;">
  <button type="submit" style="background:#c00;color:#fff;padding:12px 32px;font-size:1.2em;border:none;border-radius:6px;cursor:pointer;">Close Web Portal and Sleep</button>
</form>
<script>
let ws;
let slider, blindPos, rawPos;
function connectWS() {
  ws = new WebSocket('ws://' + window.location.host + '/ws');
  ws.onopen = function() {
    document.getElementById('disconnected').style.display = 'none';
  };
  ws.onclose = function() {
    document.getElementById('disconnected').style.display = 'block';
    setTimeout(connectWS, 2000);
  };
  ws.onerror = function() {
    document.getElementById('disconnected').style.display = 'block';
  };
  ws.onmessage = function(event) {
    let data = {};
    try { data = JSON.parse(event.data); } catch (e) {}
    updateLiveStatus(data);
  };
}

function toggleAdvanced() {
  var adv = document.getElementById('advanced');
  var toggle = document.querySelector('.adv-toggle');
  if (adv.style.display === 'none') {
    adv.style.display = 'block';
    toggle.innerHTML = 'Hide Advanced Settings &#9650;'; // ▲
  } else {
    adv.style.display = 'none';
    toggle.innerHTML = 'Show Advanced Settings &#9660;'; // ▼
  }
}

function togglePID() {
  var pid = document.getElementById('pid-settings');
  var toggle = document.querySelector('.adv-toggle[onclick="togglePID()"]');
  if (pid.style.display === 'none' || pid.style.display === '') {
    pid.style.display = 'block';
    toggle.innerHTML = 'PID Controller &#9650;'; // ▲
  } else {
    pid.style.display = 'none';
    toggle.innerHTML = 'PID Controller &#9660;'; // ▼
  }
}

function toggleStall() {
  var stall = document.getElementById('stall-settings');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleStall()"]');
  if (stall.style.display === 'none' || stall.style.display === '') {
    stall.style.display = 'block';
    toggle.innerHTML = 'Stall & Engage Detection &#9650;'; // ▲
  } else {
    stall.style.display = 'none';
    toggle.innerHTML = 'Stall & Engage Detection &#9660;'; // ▼
  }
}

function togglePWM() {
  var pwm = document.getElementById('pwm-settings');
  var toggle = document.querySelector('.adv-toggle[onclick="togglePWM()"]');
  if (pwm.style.display === 'none' || pwm.style.display === '') {
    pwm.style.display = 'block';
    toggle.innerHTML = 'Motor PWM Settings &#9650;'; // ▲
  } else {
    pwm.style.display = 'none';
    toggle.innerHTML = 'Motor PWM Settings &#9660;'; // ▼
  }
}

function toggleOTA() {
  var ota = document.getElementById('ota-section');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleOTA()"]');
  if (ota.style.display === 'none' || ota.style.display === '') {
    ota.style.display = 'block';
    toggle.innerHTML = 'Firmware Update (OTA) &#9650;'; // ▲
  } else {
    ota.style.display = 'none';
    toggle.innerHTML = 'Firmware Update (OTA) &#9660;'; // ▼
  }
}

function toggleSlaveNumber() {
  var section = document.getElementById('slave-number-section');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleSlaveNumber()"]');
  if (section.style.display === 'none' || section.style.display === '') {
    section.style.display = 'block';
    toggle.innerHTML = 'Slave Number &#9650;';
  } else {
    section.style.display = 'none';
    toggle.innerHTML = 'Slave Number &#9660;';
  }
}

function toggleKeys() {
  var keys = document.getElementById('keys-section');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleKeys()"]');
  if (keys.style.display === 'none' || keys.style.display === '') {
    keys.style.display = 'block';
    toggle.innerHTML = 'Device Keys & Gateway Configuration &#9650;'; // ▲
  } else {
    keys.style.display = 'none';
    toggle.innerHTML = 'Device Keys & Gateway Configuration &#9660;'; // ▼
  }
}

function toggleSlaves() {
  var slaves = document.getElementById('slaves-section');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleSlaves()"]');
  if (slaves.style.display === 'none' || slaves.style.display === '') {
    slaves.style.display = 'block';
    toggle.innerHTML = 'Slave Devices &#9650;'; // ▲
  } else {
    slaves.style.display = 'none';
    toggle.innerHTML = 'Slave Devices &#9660;'; // ▼
  }
}

function toggleWiFi() {
  var wifi = document.getElementById('wifi-section');
  var toggle = document.querySelector('.adv-toggle[onclick="toggleWiFi()"]');
  if (wifi.style.display === 'none' || wifi.style.display === '') {
    wifi.style.display = 'block';
    toggle.innerHTML = 'WiFi Settings &#9650;'; // ▲
  } else {
    wifi.style.display = 'none';
    toggle.innerHTML = 'WiFi Settings &#9660;'; // ▼
  }
}

// --- Track user interaction with slave sliders ---
let slaveSliderActive = {};
let slaveSliderStatus = {};
let mainSliderStatus = null;

function setSlaveSliderActive(slave, active) {
  slaveSliderActive[slave] = active;
}

function setSlaveSliderStatus(slave, status) {
  slaveSliderStatus[slave] = status;
  var statusElem = document.getElementById('slaveStatus_' + slave);
  if (statusElem) {
    if (status === 'sending') {
      statusElem.innerHTML = '<span style="color:#007bff;">&#8635;</span>'; // spinner
    } else if (status === 'ok') {
      statusElem.innerHTML = '<span style="color:#28a745;">&#10003;</span>'; // green check
      setTimeout(function() { statusElem.innerHTML = ''; }, 1000);
    } else {
      statusElem.innerHTML = '';
    }
  }
}

function setMainSliderStatus(status) {
  var sliderElem = document.getElementById('blindSlider');
  if (sliderElem) {
    if (status === 'sending') {
      sliderElem.style.border = '2px solid #007bff';
    } else if (status === 'ok') {
      sliderElem.style.border = '2px solid #28a745';
      setTimeout(function() { sliderElem.style.border = ''; }, 1000);
    } else {
      sliderElem.style.border = '';
    }
  }
}

function updateLiveStatus(data) {
  if (data.blindPos !== undefined) {
    if (!window.sliderActive) {
      slider.value = Math.round(data.blindPos / 10);
      blindPos.textContent = Math.round(data.blindPos / 10);
    }
    setMainSliderStatus('ok');
  }
  if (data.rawPos !== undefined) {
    rawPos.textContent = data.rawPos;
  }
  if (data.status !== undefined) {
    document.getElementById('blindstatus').textContent = data.status;
  }
  if (data.calib !== undefined) {
    document.getElementById('calibstate').textContent = data.calib;
  }
  // --- Update slave sliders and output fields ---
  if (Array.isArray(data.slaves)) {
    data.slaves.forEach(function(slave) {
      var sliderElem = document.getElementById('slaveSlider_' + slave.id);
      if (sliderElem) {
        // Only update if not being dragged by user
        if (!slaveSliderActive[slave.id]) {
          sliderElem.value = slave.pos;
          var outputElem = sliderElem.nextElementSibling;
          if (outputElem && outputElem.tagName === 'OUTPUT') {
            outputElem.value = slave.pos;
            outputElem.textContent = slave.pos;
          }
        }
      }
    });
  }
}

// --- Attach event listeners to slave sliders on page load/after scan ---
function attachSlaveSliderEvents() {
  var sliders = document.querySelectorAll('[id^="slaveSlider_"]');
  sliders.forEach(function(sliderElem) {
    var slaveId = sliderElem.id.replace('slaveSlider_', '');
    sliderElem.addEventListener('mousedown', function() { setSlaveSliderActive(slaveId, true); });
    sliderElem.addEventListener('touchstart', function() { setSlaveSliderActive(slaveId, true); });
    sliderElem.addEventListener('mouseup', function() {
      setSlaveSliderActive(slaveId, false);
      setSlaveSliderStatus(slaveId, 'sending');
      sendSlavePosWS(slaveId);
    });
    sliderElem.addEventListener('touchend', function() {
      setSlaveSliderActive(slaveId, false);
      setSlaveSliderStatus(slaveId, 'sending');
      sendSlavePosWS(slaveId);
    });
    sliderElem.addEventListener('mouseleave', function() { setSlaveSliderActive(slaveId, false); });
  });
}

window.onload = function() {
  slider = document.getElementById('blindSlider');
  blindPos = document.getElementById('blindpos');
  rawPos = document.getElementById('rawpos');
  connectWS();
  attachSlaveSliderEvents();

  // --- Main slider event listeners with drag protection and status ---
  window.sliderActive = false;
  slider.addEventListener('mousedown', function() { window.sliderActive = true; });
  slider.addEventListener('touchstart', function() { window.sliderActive = true; });
  slider.addEventListener('mouseup', function() {
    window.sliderActive = false;
    setMainSliderStatus('sending');
    sendMainSliderWS();
  });
  slider.addEventListener('touchend', function() {
    window.sliderActive = false;
    setMainSliderStatus('sending');
    sendMainSliderWS();
  });
  slider.addEventListener('mouseleave', function() { window.sliderActive = false; });
};

function sendMainSliderWS() {
  if (ws && ws.readyState === 1) {
    let pos = parseInt(slider.value);
    ws.send(JSON.stringify({ type: "move", pos: pos }));
  }
}

function sendSlavePosWS(slaveId) {
  var sliderElem = document.getElementById('slaveSlider_' + slaveId);
  if (ws && ws.readyState === 1 && sliderElem) {
    let pos = parseInt(sliderElem.value);
    ws.send(JSON.stringify({ type: "slave_move", id: slaveId, pos: pos }));
    setSlaveSliderStatus(slaveId, 'ok'); // Show green tick immediately after sending
  }
}

// --- Expand/collapse state persistence for all expandable sections ---
window.addEventListener('DOMContentLoaded', function() {
  const sections = [
    { id: 'advanced', toggle: 'toggleAdvanced' },
    { id: 'pid-settings', toggle: 'togglePID' },
    { id: 'stall-settings', toggle: 'toggleStall' },
    { id: 'pwm-settings', toggle: 'togglePWM' },
    { id: 'slaves-section', toggle: 'toggleSlaves' },
    { id: 'ota-section', toggle: 'toggleOTA' },
    { id: 'keys-section', toggle: 'toggleKeys' },
    { id: 'wifi-section', toggle: 'toggleWiFi' }
  ];
  sections.forEach(function(section) {
    var elem = document.getElementById(section.id);
    var toggleElem = document.querySelector('.adv-toggle[onclick="' + section.toggle + '()"]');
    if (!elem || !toggleElem) return;
    // Restore state
    if (localStorage.getItem('section_' + section.id) === 'open') {
      elem.style.display = 'block';
      // Update toggle text to show as open
      if (toggleElem.innerHTML.indexOf('&#9660;') !== -1) {
        toggleElem.innerHTML = toggleElem.innerHTML.replace('&#9660;', '&#9650;');
      }
    }
    // Save state on toggle
    toggleElem.addEventListener('click', function() {
      setTimeout(function() {
        localStorage.setItem('section_' + section.id, elem.style.display === 'block' ? 'open' : 'closed');
      }, 10);
    });
  });
});
</script>
</body></html>
)rawliteral";
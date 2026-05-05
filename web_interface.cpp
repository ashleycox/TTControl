/*
 * TT Control, advanced sinusoidal control of multi-phase turntable motors
 * Created by Ashley Cox at The Blind Man's Workshop
 * https://theblindmansworkshop.com
 * No part of this code may be used or reproduced for commercial purposes without written permission and contractual agreement
 * All external libraries and frameworks are the property of their respective authors and governed by their respective licenses
 */

#include "web_interface.h"
#include "network_manager.h"
#include "settings.h"
#include "motor.h"
#include "waveform.h"
#include "hal.h"
#include "error_handler.h"
#include "amp_monitor.h"
#include "globals.h"
#include <LittleFS.h>
#include <vector>

WebInterface webInterface;

#if NETWORK_ENABLE

static const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TT Control</title>
<style>
:root{color-scheme:light dark;--bg:#f7f7f4;--panel:#ffffff;--text:#161714;--muted:#5d625a;--line:#d8dbd2;--accent:#2364aa;--accent2:#2d8a5f;--danger:#b3261e;--warn:#8a5c00;--shadow:0 1px 2px rgba(0,0,0,.08);--focus:#ffbf47}
@media (prefers-color-scheme:dark){:root{--bg:#111412;--panel:#1b1f1c;--text:#f2f4ef;--muted:#b8beb4;--line:#343a35;--accent:#79aef2;--accent2:#70d39e;--danger:#ffb4ab;--warn:#f0c36b;--shadow:none;--focus:#ffd166}}
body[data-theme=light]{color-scheme:light;--bg:#f7f7f4;--panel:#ffffff;--text:#161714;--muted:#5d625a;--line:#d8dbd2;--accent:#2364aa;--accent2:#2d8a5f;--danger:#b3261e;--warn:#8a5c00;--shadow:0 1px 2px rgba(0,0,0,.08);--focus:#ffbf47}
body[data-theme=dark]{color-scheme:dark;--bg:#111412;--panel:#1b1f1c;--text:#f2f4ef;--muted:#b8beb4;--line:#343a35;--accent:#79aef2;--accent2:#70d39e;--danger:#ffb4ab;--warn:#f0c36b;--shadow:none;--focus:#ffd166}
body[data-theme=contrast],body.contrast{color-scheme:dark;--bg:#000;--panel:#000;--text:#fff;--muted:#e6e6e6;--line:#fff;--accent:#00c2ff;--accent2:#00d084;--danger:#ff5555;--warn:#ffd166;--shadow:none}
body[data-theme=workshop]{color-scheme:light;--bg:#f4f5f0;--panel:#ffffff;--text:#181b16;--muted:#596258;--line:#cfd7cc;--accent:#315f72;--accent2:#327a56;--danger:#a7362f;--warn:#8a6500;--shadow:0 1px 3px rgba(24,27,22,.1);--focus:#ffbf47}
body[data-theme=calm]{color-scheme:light;--bg:#f5f7f8;--panel:#ffffff;--text:#14181b;--muted:#566168;--line:#d4dde2;--accent:#285d8f;--accent2:#28736b;--danger:#b3261e;--warn:#7d6100;--shadow:0 1px 2px rgba(20,24,27,.08);--focus:#f2b84b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:16px/1.45 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}button,input,select,textarea{font:inherit}button,input,select,textarea{border:1px solid var(--line);border-radius:6px;background:var(--panel);color:var(--text)}button{min-height:44px;padding:.55rem .85rem;cursor:pointer;box-shadow:var(--shadow);display:inline-flex;align-items:center;gap:.45rem;justify-content:center}button.primary{background:var(--accent);border-color:var(--accent);color:#fff}button.good{background:var(--accent2);border-color:var(--accent2);color:#fff}button.danger{background:var(--danger);border-color:var(--danger);color:#fff}button.ghost{box-shadow:none;background:transparent}button:focus-visible,input:focus-visible,select:focus-visible,textarea:focus-visible,a:focus-visible{outline:4px solid var(--focus);outline-offset:2px}button:disabled{opacity:.55;cursor:not-allowed}body.large{font-size:19px}body.large button,body.large input,body.large select,body.large textarea{min-height:54px}.skip{position:absolute;left:.5rem;top:-5rem;background:var(--panel);border:2px solid var(--accent);padding:.7rem;z-index:20}.skip:focus{top:.5rem}.ico{font-size:.74rem;font-weight:900;letter-spacing:.02em;border:1px solid currentColor;border-radius:4px;min-width:1.7rem;text-align:center;padding:.05rem .2rem;line-height:1.35}header{position:sticky;top:0;z-index:8;background:var(--panel);border-bottom:1px solid var(--line)}.top{display:flex;gap:1rem;align-items:center;justify-content:space-between;max-width:1180px;margin:0 auto;padding:.75rem 1rem}.brand{font-weight:800;font-size:1.25rem}.net{color:var(--muted);font-size:.95rem}.header-tools{display:flex;flex-wrap:wrap;gap:.5rem;justify-content:flex-end}.toolbar-field{display:flex;align-items:center;gap:.35rem;color:var(--muted);font-size:.9rem}.toolbar-field select{width:auto;min-height:38px}.wrap{max-width:1180px;margin:0 auto;padding:1rem}.safety-bar{position:sticky;top:62px;z-index:7;background:color-mix(in srgb,var(--panel) 92%,var(--danger) 8%);border-bottom:1px solid var(--line);box-shadow:var(--shadow)}.safety-inner{max-width:1180px;margin:0 auto;padding:.65rem 1rem;display:flex;flex-wrap:wrap;gap:.5rem;align-items:center}.safety-label{font-weight:750;margin-right:.25rem}.status-grid{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:.75rem}.metric{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:.8rem;position:relative;overflow:hidden}.metric:before{content:"";position:absolute;left:0;top:0;bottom:0;width:5px;background:var(--line)}.metric.ok:before{background:var(--accent2)}.metric.warn:before{background:var(--warn)}.metric.bad:before{background:var(--danger)}.metric span{display:block;color:var(--muted);font-size:.88rem}.metric strong{display:block;font-size:1.35rem;margin-top:.15rem}.controls,.tabs,.mode-tabs,.speed-tabs,.cal-steps{display:flex;flex-wrap:wrap;gap:.5rem;margin:1rem 0}.tabs{position:sticky;top:118px;z-index:6;background:var(--bg);padding:.35rem 0;border-bottom:1px solid color-mix(in srgb,var(--line) 65%,transparent)}.tabs button[aria-selected=true],.mode-tabs button[aria-pressed=true],.speed-tabs button[aria-selected=true],.cal-steps button[aria-selected=true]{border-color:var(--accent);box-shadow:inset 0 0 0 2px var(--accent)}main section{display:none}main section.active{display:block}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:1rem;margin:1rem 0}.panel.section-head{border-left:5px solid var(--accent)}.panel legend{font-size:1.25rem;font-weight:760;padding:0 .25rem}.settings-grid,.tool-grid,.bench-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:1rem}.settings-grid fieldset{align-self:start}.settings-toolbar{position:sticky;top:176px;z-index:5}.field{display:grid;gap:.35rem;margin:.65rem 0}.field label{font-weight:650}.field small{color:var(--muted)}.field.safety{border-left:3px solid var(--warn);padding-left:.6rem}.field-head{display:flex;align-items:center;justify-content:space-between;gap:.5rem}.help-button{min-height:34px;padding:.25rem .55rem;box-shadow:none}.help-panel{background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:.65rem}.help-panel p{margin:.2rem 0}.setting-meta{color:var(--muted);font-size:.9rem}.field-error{color:var(--danger);font-weight:650;min-height:1.2em}input[aria-invalid=true],select[aria-invalid=true],textarea[aria-invalid=true]{border-color:var(--danger)}input[type=number],input[type=text],input[type=password],select,textarea{width:100%;min-height:42px;padding:.45rem .55rem}.controls input,.controls select{width:auto;min-width:7rem}input[type=checkbox]{width:1.2rem;height:1.2rem}.check-row{display:flex;align-items:center;gap:.6rem}.button-row{display:flex;flex-wrap:wrap;gap:.5rem}.sr{position:absolute;left:-10000px;width:1px;height:1px;overflow:hidden}.notice{border-left:4px solid var(--accent);padding:.7rem .9rem;background:color-mix(in srgb,var(--panel) 85%,var(--accent) 15%)}.warning{border-left-color:var(--warn)}.error{border-left-color:var(--danger)}.report{background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:.75rem;margin-top:.75rem}.report h3{margin:.1rem 0 .5rem}.report-item{border-left:4px solid var(--line);padding:.35rem .55rem;margin:.35rem 0}.report-error{border-left-color:var(--danger)}.report-warn{border-left-color:var(--warn)}.report-info{border-left-color:var(--accent)}.report-ok{border-left-color:var(--accent2)}.status-chip{display:inline-flex;align-items:center;gap:.4rem;border:1px solid var(--line);border-radius:999px;padding:.25rem .65rem;font-weight:750}.status-chip.ok{border-color:var(--accent2);color:var(--accent2)}.status-chip.warn{border-color:var(--warn);color:var(--warn)}.status-chip.bad{border-color:var(--danger);color:var(--danger)}.dash-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:.75rem}.dash-tile,.bench-card{background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:1rem}.dash-tile span,.bench-card span{color:var(--muted);display:block}.dash-tile strong{font-size:1.55rem;display:block}.bench-card textarea{min-height:12rem}.progress{height:.7rem;border-radius:999px;background:var(--bg);border:1px solid var(--line);overflow:hidden}.progress i{display:block;height:100%;background:var(--accent2);width:0}.scope{width:180px;height:180px;border:1px solid var(--line);border-radius:8px;background:var(--bg)}.chart{width:100%;height:240px;border:1px solid var(--line);border-radius:8px;background:var(--bg)}.telemetry-grid{display:grid;grid-template-columns:minmax(0,2fr) minmax(220px,1fr);gap:1rem;align-items:start}.legend{display:flex;flex-wrap:wrap;gap:.5rem;margin:.5rem 0}.legend label{border:1px solid var(--line);border-radius:999px;padding:.25rem .6rem}.simple-controls button{min-width:8rem;min-height:64px}.cal-card{display:none}.cal-card.active-cal{display:block;grid-column:1/-1}.preset-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:1rem}.preset-card{border:1px solid var(--line);border-radius:8px;padding:1rem}.diff{white-space:pre-wrap;background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:.75rem;margin-top:.75rem;max-height:14rem;overflow:auto}.log{white-space:pre-wrap;background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:1rem;min-height:8rem}.hide{display:none!important}@media (max-width:800px){.status-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.telemetry-grid{grid-template-columns:1fr}.top{align-items:flex-start;flex-direction:column}.header-tools{justify-content:flex-start}.toolbar-field{width:100%;justify-content:space-between}.toolbar-field select{width:65%}.safety-bar,.tabs{top:0}.tabs{overflow-x:auto;flex-wrap:nowrap}.tabs button{min-width:max-content}.wrap{padding:.75rem}.metric strong{font-size:1.1rem}.controls,.safety-inner,.button-row{display:grid;grid-template-columns:1fr 1fr}.controls input,.controls select{width:100%}.simple-controls .button-row{grid-template-columns:1fr}.settings-toolbar{top:0}.dash-grid,.bench-grid{grid-template-columns:1fr}}
</style>
</head>
<body>
<a class="skip" href="#mainContent">Skip to main controls</a>
<header><div class="top"><div><div class="brand">TT Control</div><div id="live" class="net" aria-live="polite" role="status">Loading</div></div><div><div class="net" id="networkLine"></div><div class="header-tools" aria-label="Web UI preferences"><label class="toolbar-field" for="homeSelect"><span>Home</span><select id="homeSelect"></select></label><label class="toolbar-field" for="themeSelect"><span>Theme</span><select id="themeSelect"><option value="system">System</option><option value="light">Light</option><option value="dark">Dark</option><option value="calm">Calm</option><option value="workshop">Workshop</option><option value="contrast">High contrast</option></select></label><button id="largeToggle" type="button">Large controls</button></div></div></div></header>
<div class="safety-bar" role="region" aria-label="Sticky safety controls">
<div class="safety-inner"><span class="safety-label">Safety</span><button class="danger" data-action="emergencyStop"><span class="ico">!</span>Emergency stop</button><button class="danger" data-action="stop"><span class="ico">STP</span>Stop</button><button data-action="toggleStandby"><span class="ico">PWR</span>Standby / wake</button><button class="good" data-action="start"><span class="ico">RUN</span>Start</button><button data-speed="0"><span class="ico">33</span>33 RPM</button><button data-speed="1"><span class="ico">45</span>45 RPM</button><button data-speed="2"><span class="ico">78</span>78 RPM</button></div>
</div>
<div class="wrap">
<div class="status-grid" aria-label="Current status">
<div class="metric"><span>State</span><strong id="state">-</strong></div>
<div class="metric"><span>Speed</span><strong id="speed">-</strong></div>
<div class="metric"><span>Frequency</span><strong id="frequency">-</strong></div>
<div class="metric"><span>Pitch</span><strong id="pitch">-</strong></div>
<div class="metric"><span>Amplifier</span><strong id="ampState">-</strong></div>
</div>
<div id="authPanel" class="notice warning hide" role="region" aria-label="Web access lock" aria-live="polite">
<strong id="authTitle">Read-only mode</strong>
<div class="button-row"><label for="pinInput">Web PIN</label><input id="pinInput" type="password" inputmode="numeric" autocomplete="current-password"><button id="unlockWeb" class="primary">Unlock controls</button><button id="lockWeb">Lock now</button></div>
</div>
<div class="controls" aria-label="Motor controls">
<button class="good" data-action="start"><span class="ico">RUN</span>Start</button>
<button class="danger" data-action="emergencyStop"><span class="ico">!</span>Emergency stop</button>
<button class="danger" data-action="stop"><span class="ico">STP</span>Stop</button>
<button data-action="toggleStandby"><span class="ico">PWR</span>Standby / wake</button>
<button data-action="cycleSpeed"><span class="ico">SPD</span>Cycle speed</button>
<button data-speed="0"><span class="ico">33</span>33 RPM</button>
<button data-speed="1"><span class="ico">45</span>45 RPM</button>
<button data-speed="2"><span class="ico">78</span>78 RPM</button>
<button data-action="resetPitch"><span class="ico">PIT</span>Reset pitch</button>
<label class="sr" for="pitchControl">Pitch percent</label><input id="pitchControl" type="number" min="-50" max="50" step="0.1" value="0" aria-label="Pitch percent">
<button id="setPitch">Set pitch</button>
<label class="sr" for="relayStage">Relay test stage</label><select id="relayStage" aria-label="Relay test stage"></select>
<button id="relayTest">Set relay test</button>
<button data-action="relayOff">Relay test off</button>
</div>
<nav class="tabs" aria-label="Primary">
<button aria-selected="true" data-tab="dashboard"><span class="ico">D</span>Dashboard</button>
<button aria-selected="false" data-tab="control"><span class="ico">C</span>Control</button>
<button aria-selected="false" data-tab="settings"><span class="ico">S</span>Settings</button>
<button aria-selected="false" data-tab="calibrate"><span class="ico">T</span>Calibrate</button>
<button aria-selected="false" data-tab="network"><span class="ico">N</span>Network</button>
<button aria-selected="false" data-tab="presets"><span class="ico">P</span>Presets</button>
<button aria-selected="false" data-tab="bench"><span class="ico">B</span>Bench</button>
<button aria-selected="false" data-tab="diagnostics"><span class="ico">I</span>Diagnostics</button>
<button aria-selected="false" data-tab="errors"><span class="ico">E</span>Errors</button>
</nav>
<main id="mainContent" tabindex="-1">
<section id="dashboard" class="active">
<div class="mode-tabs" aria-label="Dashboard modes">
<button data-mode="standard" aria-pressed="true">Standard</button>
<button data-mode="stats" aria-pressed="false">Stats</button>
<button data-mode="dim" aria-pressed="false">Dim</button>
<button data-mode="scope" aria-pressed="false">Scope</button>
</div>
<div id="dashboardBody" class="panel"></div>
</section>
<section id="control">
<div class="panel simple-controls">
<h2>Simple control</h2>
<div class="button-row"><button class="good" data-action="start"><span class="ico">RUN</span>Start</button><button class="danger" data-action="emergencyStop"><span class="ico">!</span>Emergency stop</button><button class="danger" data-action="stop"><span class="ico">STP</span>Stop</button><button data-action="toggleStandby"><span class="ico">PWR</span>Standby / wake</button></div>
<div class="button-row"><button data-speed="0"><span class="ico">33</span>33 RPM</button><button data-speed="1"><span class="ico">45</span>45 RPM</button><button data-speed="2"><span class="ico">78</span>78 RPM</button><button data-action="resetPitch"><span class="ico">PIT</span>Reset pitch</button></div>
<div class="field"><label for="simplePitch">Pitch percent</label><input id="simplePitch" type="number" min="-50" max="50" step="0.1" value="0"></div><button id="setSimplePitch">Set pitch</button>
</div>
</section>
<section id="settings">
<div class="panel section-head settings-toolbar">
<div id="settingsDirty" class="notice warning hide" aria-live="polite"></div>
<div class="field"><label for="settingsSearch">Search settings</label><input id="settingsSearch" type="text" placeholder="Search by label or help text"></div>
<div id="settingsReview" class="diff hide" aria-live="polite"></div>
<div class="button-row"><button class="primary" id="applySave">Save settings</button><button id="applyOnly">Apply without saving</button><button id="reviewSettings">Review changes</button><button id="discardSettings">Discard changes</button><button class="danger" id="factoryReset">Factory reset</button><button id="resetRuntime">Reset runtime</button></div>
</div>
<div id="globalSettings" class="settings-grid"></div>
<div class="panel">
<h2>Speed settings</h2>
<div class="speed-tabs"><button data-speed-tab="0" aria-selected="true">33 RPM</button><button data-speed-tab="1" aria-selected="false">45 RPM</button><button data-speed-tab="2" aria-selected="false">78 RPM</button></div>
<div id="speedSettings" class="settings-grid"></div>
</div>
</section>
<section id="calibrate">
<div class="panel section-head">
<h2>Calibration workflow</h2>
<div id="calStepper" class="cal-steps" aria-label="Calibration steps"></div>
</div>
<div class="tool-grid" id="calibrationTools"></div>
</section>
<section id="network">
<div class="panel"><div id="networkStatus"></div><div id="networkDirty" class="notice warning hide" aria-live="polite"></div><div class="button-row"><button class="primary" id="saveNetwork">Save and reconnect</button><button id="discardNetwork">Discard changes</button><button id="scanNetworks">Scan networks</button></div></div>
<div id="networkForm" class="settings-grid"></div>
<div id="scanResults" class="panel"></div>
</section>
<section id="presets">
<div class="panel"><h2>Compare preset slots</h2><div class="button-row"><select id="compareA" aria-label="First preset slot"></select><select id="compareB" aria-label="Second preset slot"></select><button id="comparePresets">Compare</button></div><div id="presetCompare" class="diff hide" aria-live="polite"></div></div>
<div class="preset-grid" id="presetGrid"></div>
</section>
<section id="bench">
<div id="benchBody"></div>
</section>
<section id="diagnostics">
<div id="diagnosticsBody" class="panel"></div>
<div class="panel"><h2>Recent events</h2><div id="eventFeed" class="log"></div></div>
<div class="panel"><h2>Backup</h2><div class="button-row"><button id="exportBackup">Export full backup</button><button id="validateBackup">Validate import</button><button id="importBackup" class="primary">Import backup</button></div><textarea id="backupText" rows="10" aria-label="Backup JSON"></textarea><div id="backupReport" class="report hide" aria-live="polite"></div></div>
</section>
<section id="errors">
<div class="panel"><button id="clearErrors">Clear error log</button></div>
<div id="errorLog" class="log"></div>
</section>
</main>
</div>
<script>
let speedNames=["33 RPM","45 RPM","78 RPM"],options={},globalGroups=[],speedFields=[],networkFields=[];
let statusData=null,settingsData=null,networkData=null,presetsData=null,diagnosticsData=null,baselineSettings=null,baselineNetwork=null,authState=null,currentTab="dashboard",dashboardMode="standard",speedTab=0;
let authToken="";
let deviceHomePage=0;
let lastState="",lastSpeed=-1,lastAmpState="";
let activeCalStep=0;
const telemetrySeries={frequency:true,pitch:true,amp:true};
const telemetry=[],events=[],MAX_TELEMETRY=180,MAX_EVENTS=60;
const homeTabs=["dashboard","control","settings","calibrate","network","presets","bench","diagnostics","errors"];
const tabNames={dashboard:"Dashboard",control:"Control",settings:"Settings",calibrate:"Calibrate",network:"Network",presets:"Presets",bench:"Bench",diagnostics:"Diagnostics",errors:"Errors"};
const presetGlobalMap={pm:"phaseMode",maxAmp:"maxAmplitude",ssCurve:"softStartCurve",fda:"freqDependentAmplitude",vfLF:"vfLowFreq",vfLB:"vfLowBoost",vfMF:"vfMidFreq",vfMB:"vfMidBoost",brkMd:"brakeMode",brkDur:"brakeDuration",brkPG:"brakePulseGap",brkSF:"brakeStartFreq",brkStF:"brakeStopFreq",brkCut:"softStopCutoff"};
const presetSpeedMap={f:"frequency",minF:"minFrequency",maxF:"maxFrequency",ssD:"softStartDuration",rAmp:"reducedAmplitude",aDly:"amplitudeDelay",kick:"startupKick",kDur:"startupKickDuration",kRmp:"startupKickRampDuration",fTyp:"filterType",iir:"iirAlpha",fir:"firProfile"};
const $=id=>document.getElementById(id);
function esc(v){return String(v??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]))}
function clone(v){return JSON.parse(JSON.stringify(v))}
async function api(path,opts={}){opts.headers=Object.assign({},opts.headers||{});if(authToken)opts.headers["X-TTControl-Token"]=authToken;const r=await fetch(path,opts);const t=await r.text();let data={};try{data=t?JSON.parse(t):{}}catch(e){throw new Error(t||r.statusText)}if(!r.ok){if(r.status===401)loadAuth();throw new Error(data.error||r.statusText)}return data}
function setLive(msg){$("live").textContent=msg}
function statusClass(state){state=String(state||"").toUpperCase();if(state.includes("RUN"))return"ok";if(state.includes("START")||state.includes("RAMP")||state.includes("BRAK"))return"warn";if(state.includes("STOP")||state.includes("STANDBY"))return"warn";return""}
function addEvent(msg){events.unshift(`${new Date().toLocaleTimeString()} ${msg}`);while(events.length>MAX_EVENTS)events.pop();renderEventFeed()}
function renderEventFeed(){const box=$("eventFeed");if(box)box.textContent=events.join("\n")||"No recent events"}
function is78Enabled(){return !(settingsData&&settingsData.global&&settingsData.global.enable78rpm===false)}
function disabled78Message(){return"78 RPM is disabled. Enable 78 RPM in Settings before selecting it."}
function sync78Controls(locked=false){const disabled=!is78Enabled();document.querySelectorAll('button[data-speed="2"],button[data-bench-speed="2"],button[data-speed-tab="2"]').forEach(el=>{el.hidden=disabled;el.disabled=!!locked||disabled;el.title=disabled?disabled78Message():""});document.querySelectorAll("select").forEach(el=>{if(!el.id||!el.id.startsWith("cal"))return;[...el.options].forEach(o=>{if(Number(o.value)===2)o.hidden=disabled});if(disabled&&Number(el.value)===2)el.value="0"})}
function setLockedUI(){const locked=authState&&authState.required&&!authState.unlocked;document.querySelectorAll("[data-action],button[data-speed],[data-bench],[data-bench-speed],#homeSelect,#setPitch,#setSimplePitch,#benchSetPitch,#relayTest,#applySave,#applyOnly,#factoryReset,#resetRuntime,#saveNetwork,#clearErrors,#importBackup").forEach(el=>{if(el)el.disabled=!!locked});document.querySelectorAll("[data-pa]").forEach(el=>{if(el)el.disabled=!!(locked&&!["preview","previewImport","export"].includes(el.dataset.pa))});sync78Controls(!!locked)}
function renderAuth(){const p=$("authPanel");if(!p||!authState){return}if(!authState.required){p.classList.add("hide");setLockedUI();return}p.classList.remove("hide");$("authTitle").textContent=authState.unlocked?"Controls unlocked":"Read-only mode: enter PIN to change controls or settings";$("unlockWeb").hidden=!!authState.unlocked;$("lockWeb").hidden=!authState.unlocked;setLockedUI()}
async function loadAuth(){authState=await api("/api/auth");renderAuth()}
async function unlockWeb(){const res=await api("/api/auth",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({pin:$("pinInput").value})});authToken=res.token||"";try{sessionStorage.setItem("ttc_token",authToken)}catch(e){}authState=res;$("pinInput").value="";renderAuth();addEvent("Web controls unlocked")}
async function lockWeb(){await api("/api/auth",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({action:"logout"})});authToken="";try{sessionStorage.removeItem("ttc_token")}catch(e){}await loadAuth();addEvent("Web controls locked")}
function inputId(scope,key,index){return `${scope}_${index??"g"}_${key}`}
function globalFields(){return globalGroups.flatMap(g=>g[1]||[])}
function allSettingFields(){return [...globalFields(),...speedFields]}
function findField(fields,key){return fields.find(f=>f.k===key)}
function optionLabel(key,value){const found=(options[key]||[]).find(o=>Number(o[0])===Number(value));return found?found[1]:String(value)}
function pathLabel(path){const parts=path.split(".");if(parts[0]==="global"){const f=findField(globalFields(),parts[1]);return f?f.l:path}if(parts[0]==="network"){if(parts[1]==="apPasswordSet")return "Setup AP password state";const f=findField(networkFields,parts[1]);return f?f.l:path}if(parts[0]==="speeds"){const index=Number(parts[1]);if(parts[2]==="phaseOffset")return `${speedNames[index]} phase ${Number(parts[3])+1} offset`;const f=findField(speedFields,parts[2]);return f?`${speedNames[index]} ${f.l}`:path}return path}
function isSafetyPath(path){const parts=path.split(".");if(parts[0]==="global"){const f=findField(globalFields(),parts[1]);return !!(f&&f.safety)}if(parts[0]==="speeds"){const key=parts[2]==="phaseOffset"?`phase${parts[3]}`:parts[2];const f=findField(speedFields,key);return !!(f&&f.safety)}return false}
function flatten(obj,prefix="",out={}){if(Array.isArray(obj)){obj.forEach((v,i)=>flatten(v,prefix?`${prefix}.${i}`:String(i),out));return out}if(obj&&typeof obj==="object"){Object.keys(obj).forEach(k=>flatten(obj[k],prefix?`${prefix}.${k}`:k,out));return out}out[prefix]=obj;return out}
function sameValue(a,b){if(typeof a==="number"||typeof b==="number")return Math.abs(Number(a)-Number(b))<0.0001;return String(a)===String(b)}
function diffs(from,to){const a=flatten(from),b=flatten(to),keys=new Set([...Object.keys(a),...Object.keys(b)]),out=[];keys.forEach(k=>{if(!sameValue(a[k],b[k]))out.push({path:k,from:a[k],to:b[k]})});return out}
function displayValue(path,value){if(typeof value==="boolean")return value?"on":"off";const parts=path.split(".");let f=null;if(parts[0]==="global")f=findField(globalFields(),parts[1]);if(parts[0]==="network")f=findField(networkFields,parts[1]);if(parts[0]==="speeds"){const key=parts[2]==="phaseOffset"?`phase${parts[3]}`:parts[2];f=findField(speedFields,key)}if(f&&f.t==="select")return optionLabel(f.o,value);return String(value)}
function validIpAddr(v){return /^(\d{1,3}\.){3}\d{1,3}$/.test(v)&&v.split(".").every(x=>Number(x)>=0&&Number(x)<=255)}
function fieldHelpMeta(field){const bits=[];if(field.unit)bits.push(`Unit: ${field.unit}`);if(field.t==="number")bits.push(`Range: ${field.min} to ${field.max}, step ${field.step}`);if(field.t==="select")bits.push(`Choices: ${(options[field.o]||[]).map(o=>o[1]).join(", ")}`);if(field.maxLength)bits.push(`Maximum length: ${field.maxLength}`);if(field.t==="password"&&field.minLength)bits.push(`Minimum length when changed: ${field.minLength}`);bits.push(`Key: ${field.k}`);return bits}
function fieldHelpHtml(id,field){const meta=fieldHelpMeta(field),help=field.help||"No extra guidance is available for this setting.";return `<div id="${id}_detail" class="help-panel hide"><p>${esc(help)}</p>${field.safety?`<p><strong>Safety-related:</strong> changes can affect motor drive, relay state, or output behavior.</p>`:""}<p class="setting-meta">${esc(meta.join(" | "))}</p></div>`}
function toggleFieldHelp(button){const panel=$(button.dataset.help);if(!panel)return;const open=panel.classList.toggle("hide")===false;button.setAttribute("aria-expanded",String(open))}
function makeField(scope,field,value,index){const id=inputId(scope,field.k,index);const wrap=document.createElement("div");wrap.className=`field${field.safety?" safety":""}`;wrap.dataset.search=`${field.l||""} ${field.help||""} ${field.k||""}`.toLowerCase();const helpButton=`<button type="button" class="help-button" data-help="${id}_detail" aria-controls="${id}_detail" aria-expanded="false">Help</button>`;const help=fieldHelpHtml(id,field);const err=`<div id="${id}_err" class="field-error" aria-live="polite"></div>`;const described=`aria-describedby="${id}_detail ${id}_err" aria-invalid="false"`;if(field.t==="checkbox"){wrap.innerHTML=`<div class="field-head"><label class="check-row"><input id="${id}" type="checkbox" ${described}${value?" checked":""}> <span>${esc(field.l)}</span></label>${helpButton}</div>${help}${err}`;wrap.querySelector("[data-help]").onclick=e=>toggleFieldHelp(e.currentTarget);return wrap}let control="";if(field.t==="select"){control=`<select id="${id}" ${described}>${(options[field.o]||[]).map(([v,l])=>`<option value="${v}"${Number(value)===Number(v)?" selected":""}>${esc(l)}</option>`).join("")}</select>`}else{const type=field.t==="password"?"password":field.t==="text"?"text":"number";let attrs=` type="${type}" value="${esc(value??"")}" ${described}`;if(field.t==="number")attrs+=` min="${field.min}" max="${field.max}" step="${field.step}"`;if(field.maxLength)attrs+=` maxlength="${field.maxLength}"`;if(field.t==="password")attrs+=` autocomplete="new-password"`;control=`<input id="${id}"${attrs}>`}wrap.innerHTML=`<div class="field-head"><label for="${id}">${esc(field.l)}</label>${helpButton}</div>${control}${help}${err}`;wrap.querySelector("[data-help]").onclick=e=>toggleFieldHelp(e.currentTarget);return wrap}
function fieldValue(scope,field,index){const el=$(inputId(scope,field.k,index));if(!el)return null;if(field.t==="checkbox")return el.checked;if(field.t==="number"||field.t==="select")return Number(el.value);return el.value}
function setFieldError(id,message,show=true){const el=$(id),err=$(`${id}_err`);if(el)el.setAttribute("aria-invalid",message?"true":"false");if(err&&show)err.textContent=message||"";return !message}
function validateOne(scope,field,index,show=true){const id=inputId(scope,field.k,index),el=$(id);if(!el)return true;let message="";if(field.t==="number"){if(el.value==="")message="Required.";else{const n=Number(el.value);if(!Number.isFinite(n))message="Enter a number.";else if(field.min!==undefined&&n<Number(field.min))message=`Minimum is ${field.min}.`;else if(field.max!==undefined&&n>Number(field.max))message=`Maximum is ${field.max}.`}}else if(field.maxLength&&el.value.length>field.maxLength)message=`Maximum length is ${field.maxLength} characters.`;if(field.format==="ip"&&el.value&&!validIpAddr(el.value.trim()))message="Use an IPv4 address like 192.168.1.50.";if(field.t==="password"&&el.value&&field.minLength&&el.value.length<field.minLength)message=`Use at least ${field.minLength} characters, or leave blank if allowed.`;return setFieldError(id,message,show)}
function validateSettingsForm(show=true){let ok=true;globalFields().forEach(f=>{ok=validateOne("global",f,undefined,show)&&ok});speedFields.forEach(f=>{ok=validateOne("speed",f,speedTab,show)&&ok});if(!ok)return false;captureSpeedTab();const s=settingsData.speeds[speedTab];const min=s.minFrequency,max=s.maxFrequency,f=s.frequency;if(min>max){setFieldError(inputId("speed","minFrequency",speedTab),"Minimum must not exceed maximum.",show);setFieldError(inputId("speed","maxFrequency",speedTab),"Maximum must be at least minimum.",show);ok=false}if(f<min||f>max){setFieldError(inputId("speed","frequency",speedTab),"Frequency must be between minimum and maximum.",show);ok=false}const g=currentSettings(false).global;if(g.ampTempWarnC!==undefined&&g.ampTempShutdownC!==undefined&&g.ampTempWarnC>=g.ampTempShutdownC){setFieldError(inputId("global","ampTempWarnC"),"Warning must be below shutdown.",show);setFieldError(inputId("global","ampTempShutdownC"),"Shutdown must be above warning.",show);ok=false}return ok}
function validateNetworkForm(show=true){let ok=true;networkFields.forEach(f=>{ok=validateOne("network",f,undefined,show)&&ok});if(!ok)return false;const cfg=gatherNetwork();if(Number(cfg.mode)!==0&&!cfg.ssid){setFieldError(inputId("network","ssid"),"SSID is required for station mode.",show);ok=false}if(!cfg.dhcp){["staticIp","gateway","subnet","dns"].forEach(k=>{if(!cfg[k]){setFieldError(inputId("network",k),"Required when DHCP is off.",show);ok=false}})}if(!cfg.apSsid){setFieldError(inputId("network","apSsid"),"Setup AP name is required.",show);ok=false}return ok}
function newReport(){return{errors:0,warnings:0,infos:0,items:[]}}
function reportIssue(r,kind,path,msg){if(kind==="error")r.errors++;else if(kind==="warn")r.warnings++;else r.infos++;r.items.push({kind,path,msg})}
function validateImportField(r,field,value,path,required=false){if(!field){reportIssue(r,"warn",path,"This field is not available in this firmware.");return}if(value===undefined||value===null||value===""){if(required)reportIssue(r,"error",path,"Required value is missing.");return}if(field.t==="checkbox"&&typeof value!=="boolean")reportIssue(r,"error",path,"Expected on/off value.");if((field.t==="number"||field.t==="select")&&typeof value!=="number")reportIssue(r,"error",path,"Expected a JSON number.");if((field.t==="text"||field.t==="password")&&typeof value!=="string")reportIssue(r,"error",path,"Expected text.");if((field.t==="number"||field.t==="select")&&typeof value==="number"){if(!Number.isFinite(value))reportIssue(r,"error",path,"Number is not finite.");else{if(field.min!==undefined&&value<Number(field.min))reportIssue(r,"error",path,`Below minimum ${field.min}.`);if(field.max!==undefined&&value>Number(field.max))reportIssue(r,"error",path,`Above maximum ${field.max}.`)}}if(field.t==="select"&&typeof value==="number"&&(options[field.o]||[]).length&&!options[field.o].some(o=>Number(o[0])===Number(value)))reportIssue(r,"error",path,"Unknown choice value.");if(field.maxLength&&typeof value==="string"&&value.length>field.maxLength)reportIssue(r,"error",path,`Longer than ${field.maxLength} characters.`);if(field.format==="ip"&&typeof value==="string"&&value&& !validIpAddr(value.trim()))reportIssue(r,"error",path,"Invalid IPv4 address.");if(field.t==="password"&&typeof value==="string"&&value&&field.minLength&&value.length<field.minLength)reportIssue(r,"error",path,`Use at least ${field.minLength} characters, or leave it blank to keep the saved value.`)}
function mergeReportInto(target,source,prefix){source.items.forEach(i=>reportIssue(target,i.kind,prefix?`${prefix}: ${i.path}`:i.path,i.msg))}
function validateSpeedRelationship(r,s,path){if(!s)return;const min=s.minFrequency??s.minF,max=s.maxFrequency??s.maxF,f=s.frequency??s.f;if(typeof min==="number"&&typeof max==="number"&&min>max)reportIssue(r,"error",path,"Minimum frequency is higher than maximum frequency.");if(typeof f==="number"&&typeof min==="number"&&typeof max==="number"&&(f<min||f>max))reportIssue(r,"error",path,"Frequency is outside this speed's minimum and maximum range.")}
function validateSettingsImportObject(obj){const r=newReport();if(!obj||typeof obj!=="object"||Array.isArray(obj)){reportIssue(r,"error","settings","Settings import must be a JSON object.");return r}const g=obj.global;if(!g||typeof g!=="object"||Array.isArray(g))reportIssue(r,"error","settings.global","Global settings object is missing.");else{Object.keys(g).forEach(k=>{if(k!=="schemaVersion"&&!findField(globalFields(),k)&&k!=="totalRuntime")reportIssue(r,"warn",`settings.global.${k}`,"This setting is not used by this firmware.");});globalFields().forEach(f=>{if(Object.prototype.hasOwnProperty.call(g,f.k))validateImportField(r,f,g[f.k],`settings.global.${f.k}`)})}const speeds=obj.speeds;if(!Array.isArray(speeds))reportIssue(r,"error","settings.speeds","Speeds must be an array.");else{if(speeds.length!==3)reportIssue(r,"warn","settings.speeds","Expected three speed entries.");speeds.slice(0,3).forEach((s,i)=>{if(!s||typeof s!=="object"||Array.isArray(s)){reportIssue(r,"error",`settings.speeds.${i}`,"Speed entry must be an object.");return}Object.keys(s).forEach(k=>{if(k!=="phaseOffset"&&!findField(speedFields,k))reportIssue(r,"warn",`settings.speeds.${i}.${k}`,"This speed setting is not used by this firmware.");});speedFields.forEach(f=>{if(f.k.startsWith("phase"))return;if(Object.prototype.hasOwnProperty.call(s,f.k))validateImportField(r,f,s[f.k],`settings.speeds.${i}.${f.k}`)});if(Object.prototype.hasOwnProperty.call(s,"phaseOffset")){if(!Array.isArray(s.phaseOffset))reportIssue(r,"error",`settings.speeds.${i}.phaseOffset`,"Phase offsets must be an array.");else s.phaseOffset.slice(0,4).forEach((v,p)=>validateImportField(r,findField(speedFields,`phase${p}`),v,`settings.speeds.${i}.phaseOffset.${p}`))}validateSpeedRelationship(r,s,`settings.speeds.${i}`)})}return r}
function validateNetworkImportObject(cfg){const r=newReport();if(!cfg||typeof cfg!=="object"||Array.isArray(cfg)){reportIssue(r,"error","network.config","Network config must be a JSON object.");return r}Object.keys(cfg).forEach(k=>{if(!["passwordSet","apPasswordSet","webPinSet"].includes(k)&&!findField(networkFields,k))reportIssue(r,"warn",`network.config.${k}`,"This network setting is not used by this firmware.");});networkFields.forEach(f=>{if(Object.prototype.hasOwnProperty.call(cfg,f.k))validateImportField(r,f,cfg[f.k],`network.config.${f.k}`)});if(Number(cfg.mode)!==0&&!cfg.ssid)reportIssue(r,"error","network.config.ssid","SSID is required for station mode.");if(cfg.dhcp===false)["staticIp","gateway","subnet","dns"].forEach(k=>{if(!cfg[k])reportIssue(r,"error",`network.config.${k}`,"Required when DHCP is off.")});if(!cfg.apSsid)reportIssue(r,"error","network.config.apSsid","Setup AP SSID is required.");return r}
function validatePresetImportObject(obj){const r=newReport();if(!obj||typeof obj!=="object"||Array.isArray(obj)){reportIssue(r,"error","preset","Preset import must be a JSON object.");return r}Object.keys(obj).forEach(k=>{if(k!=="speeds"&&!presetGlobalMap[k])reportIssue(r,"warn",k,"This preset key is not used by this firmware.");});Object.keys(presetGlobalMap).forEach(k=>{if(Object.prototype.hasOwnProperty.call(obj,k)){const field=findField(globalFields(),presetGlobalMap[k]);if(field)validateImportField(r,field,obj[k],k)}});if(Object.prototype.hasOwnProperty.call(obj,"speeds")){if(!Array.isArray(obj.speeds))reportIssue(r,"error","speeds","Speeds must be an array.");else{if(obj.speeds.length>3)reportIssue(r,"warn","speeds","Only the first three speed entries are imported.");obj.speeds.slice(0,3).forEach((s,i)=>{if(!s||typeof s!=="object"||Array.isArray(s)){reportIssue(r,"error",`speeds.${i}`,"Speed entry must be an object.");return}Object.keys(s).forEach(k=>{if(k!=="ph"&&!presetSpeedMap[k])reportIssue(r,"warn",`speeds.${i}.${k}`,"This preset speed key is not used by this firmware.");});Object.keys(presetSpeedMap).forEach(k=>{if(Object.prototype.hasOwnProperty.call(s,k)){const field=findField(speedFields,presetSpeedMap[k]);if(field)validateImportField(r,field,s[k],`speeds.${i}.${k}`)}});if(Object.prototype.hasOwnProperty.call(s,"ph")){if(!Array.isArray(s.ph))reportIssue(r,"error",`speeds.${i}.ph`,"Phase offsets must be an array.");else s.ph.slice(0,4).forEach((v,p)=>validateImportField(r,findField(speedFields,`phase${p}`),v,`speeds.${i}.ph.${p}`))}validateSpeedRelationship(r,s,`speeds.${i}`)})}}else reportIssue(r,"warn","speeds","No speed entries are included; missing fields will stay unchanged in the target preset.");return r}
function validateBackupObject(b){const r=newReport();if(!b||typeof b!=="object"||Array.isArray(b)){reportIssue(r,"error","backup","Backup import must be a JSON object.");return r}if(b.settings)mergeReportInto(r,validateSettingsImportObject(b.settings),"settings");else reportIssue(r,"warn","settings","No motor settings are included.");if(b.network&&b.network.config){mergeReportInto(r,validateNetworkImportObject(b.network.config),"network");reportIssue(r,"info","network","Wi-Fi passwords and the web PIN are intentionally not imported from backups.");}if(Array.isArray(b.presets)){b.presets.forEach((p,i)=>{if(!p||typeof p!=="object"){reportIssue(r,"error",`presets.${i}`,"Preset entry must be an object.");return}if(typeof p.slot!=="number"||p.slot<0||p.slot>=5)reportIssue(r,"error",`presets.${i}.slot`,"Preset slot must be 0 through 4.");if(p.json){try{mergeReportInto(r,validatePresetImportObject(JSON.parse(p.json)),`presets.${i}.json`)}catch(e){reportIssue(r,"error",`presets.${i}.json`,"Preset JSON is not valid.")}}})}else if(b.presets!==undefined)reportIssue(r,"error","presets","Presets must be an array.");if(!r.errors&&!r.warnings)reportIssue(r,"info","import","Validation passed with no issues.");return r}
function renderReport(el,title,r,extra=""){if(!el)return;el.classList.remove("hide");const summary=`${r.errors} error${r.errors===1?"":"s"}, ${r.warnings} warning${r.warnings===1?"":"s"}, ${r.infos} note${r.infos===1?"":"s"}`;el.innerHTML=`<h3>${esc(title)}</h3><p>${esc(summary)}</p>${extra}${r.items.map(i=>`<div class="report-item report-${i.kind==="warn"?"warn":i.kind}"><strong>${esc(i.kind==="warn"?"Warning":i.kind==="error"?"Error":"Note")}:</strong> ${esc(i.path)} - ${esc(i.msg)}</div>`).join("")}`}
function speedComparable(s){return{frequency:s.frequency,minFrequency:s.minFrequency,maxFrequency:s.maxFrequency,phaseOffset:[...s.phaseOffset],softStartDuration:s.softStartDuration,reducedAmplitude:s.reducedAmplitude,amplitudeDelay:s.amplitudeDelay,startupKick:s.startupKick,startupKickDuration:s.startupKickDuration,startupKickRampDuration:s.startupKickRampDuration,filterType:s.filterType,iirAlpha:s.iirAlpha,firProfile:s.firProfile}}
function settingsComparable(data){const g={};globalFields().forEach(f=>{g[f.k]=data.global[f.k]});return{global:g,speeds:data.speeds.map(speedComparable)}}
function currentSettings(save=false){captureSpeedTab();const g={};globalFields().forEach(f=>{g[f.k]=fieldValue("global",f)});return{save,global:g,speeds:settingsData.speeds}}
function networkComparableFromConfig(c){const out={};networkFields.forEach(f=>{if(f.k!=="password"&&f.k!=="apPassword")out[f.k]=c[f.k]});out.password="";out.apPassword="";out.apPasswordSet=!!c.apPasswordSet;return out}
function currentNetworkComparable(){const out=gatherNetwork();const station=$(inputId("network","password"))?.value||"",ap=$(inputId("network","apPassword"))?.value||"";out.password=station?"<changed>":"";out.apPassword=ap?"<changed>":"";out.apPasswordSet=!!ap;return out}
function updateSettingsDirty(){const box=$("settingsDirty");if(!box||!baselineSettings||!settingsData){return}const d=diffs(baselineSettings,settingsComparable(currentSettings(false)));if(!d.length){box.classList.add("hide");box.textContent="";return}const safety=d.filter(x=>isSafetyPath(x.path)).map(x=>pathLabel(x.path));box.classList.remove("hide");box.innerHTML=`<strong>${d.length} unsaved setting ${d.length===1?"change":"changes"}.</strong>${safety.length?`<br>Safety-related changes: ${esc([...new Set(safety)].slice(0,8).join(", "))}`:""}`}
function updateNetworkDirty(){const box=$("networkDirty");if(!box||!baselineNetwork||!networkData)return;const d=diffs(baselineNetwork,currentNetworkComparable());if(!d.length){box.classList.add("hide");box.textContent="";return}box.classList.remove("hide");box.innerHTML=`<strong>${d.length} unsaved network ${d.length===1?"change":"changes"}.</strong>`}
function wireForm(root,fn){root.querySelectorAll("input,select,textarea").forEach(el=>{el.oninput=fn;el.onchange=fn})}
function renderSettings(){if(!settingsData)return;const root=$("globalSettings");root.innerHTML="";for(const [title,fields] of globalGroups){const p=document.createElement("fieldset");p.className="panel";p.innerHTML=`<legend>${esc(title)}</legend>`;for(const f of fields)p.appendChild(makeField("global",f,settingsData.global[f.k]));root.appendChild(p)}renderSpeedSettings();wireForm($("settings"),()=>{validateSettingsForm(false);updateSettingsDirty()});updateSettingsDirty();filterSettings();setLockedUI()}
function filterSettings(){const q=($("settingsSearch")?.value||"").trim().toLowerCase();document.querySelectorAll("#settings .field[data-search]").forEach(el=>el.classList.toggle("hide",q&&!el.dataset.search.includes(q)));document.querySelectorAll("#settings fieldset.panel").forEach(p=>{const visible=[...p.querySelectorAll(".field")].some(f=>!f.classList.contains("hide"));p.classList.toggle("hide",q&&!visible)})}
function reviewSettingsChanges(){if(!baselineSettings||!settingsData)return "";const d=diffs(baselineSettings,settingsComparable(currentSettings(false)));const text=d.length?d.map(x=>`${isSafetyPath(x.path)?"[SAFETY] ":""}${pathLabel(x.path)}: ${displayValue(x.path,x.from)} -> ${displayValue(x.path,x.to)}`).join("\n"):"No pending setting changes.";const box=$("settingsReview");if(box){box.classList.remove("hide");box.textContent=text}return text}
function captureSpeedTab(){if(!settingsData||!$("speedSettings").children.length)return;const s=settingsData.speeds[speedTab];for(const f of speedFields){if(f.k.startsWith("phase"))s.phaseOffset[Number(f.k.slice(5))]=fieldValue("speed",f,speedTab);else s[f.k]=fieldValue("speed",f,speedTab)}}
function renderSpeedSettings(){if(!settingsData)return;if(!is78Enabled()&&speedTab===2)speedTab=0;document.querySelectorAll("[data-speed-tab]").forEach(b=>b.setAttribute("aria-selected",String(Number(b.dataset.speedTab)===speedTab)));const root=$("speedSettings");root.innerHTML="";const p=document.createElement("fieldset");p.className="panel";p.innerHTML=`<legend>${esc(speedNames[speedTab])}</legend>`;for(const f of speedFields){let value=f.k.startsWith("phase")?settingsData.speeds[speedTab].phaseOffset[Number(f.k.slice(5))]:settingsData.speeds[speedTab][f.k];p.appendChild(makeField("speed",f,value,speedTab))}root.appendChild(p);wireForm($("settings"),()=>{validateSettingsForm(false);updateSettingsDirty()});updateSettingsDirty();filterSettings();sync78Controls(authState&&authState.required&&!authState.unlocked)}
async function loadSchema(){const s=await api("/api/schema");speedNames=s.speedNames||speedNames;options=s.options||{};globalGroups=s.globalGroups||[];speedFields=s.speedFields||[];networkFields=s.networkFields||[]}
async function loadSettings(){settingsData=await api("/api/settings");baselineSettings=settingsComparable(settingsData);renderSettings();renderCalibrationTools()}
async function applySettings(save){if(!validateSettingsForm(true)){setLive("Fix setting errors before applying");return}if(save){const review=reviewSettingsChanges();if(review&&review!=="No pending setting changes."&&!confirm(`Save these changes?\n\n${review.slice(0,1200)}`))return}settingsData=await api("/api/settings",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(currentSettings(save))});baselineSettings=settingsComparable(settingsData);renderSettings();addEvent(save?"Settings saved":"Settings applied");setLive(save?"Settings saved":"Settings applied")}
function speedSelect(id){return `<select id="${id}">${speedNames.map((n,i)=>`<option value="${i}"${i===2&&!is78Enabled()?" hidden":""}>${n}</option>`).join("")}</select>`}
function renderCalStepper(){const names=["Frequency","Phase","Kick","Brake","Amplitude"],stepper=$("calStepper");if(!stepper)return;stepper.innerHTML=names.map((n,i)=>`<button data-cal-step="${i}" aria-selected="${i===activeCalStep}"><span class="ico">${i+1}</span>${n}</button>`).join("");stepper.querySelectorAll("[data-cal-step]").forEach(b=>b.onclick=()=>{activeCalStep=Number(b.dataset.calStep);renderCalStepper();document.querySelectorAll("#calibrationTools .cal-card").forEach((p,i)=>p.classList.toggle("active-cal",i===activeCalStep))})}
function renderCalibrationTools(){if(!settingsData)return;const root=$("calibrationTools");if(!root)return;root.innerHTML=`<div class="panel cal-card"><h2>Speed frequency</h2><div class="field"><label for="calSpeed">Speed</label>${speedSelect("calSpeed")}</div><div class="field"><label for="calFreq">Frequency Hz</label><input id="calFreq" type="number" min="10" max="1500" step="0.1"></div><div class="button-row"><button data-cal="frequency">Apply</button><button data-cal="frequencySave" class="primary">Save</button></div></div><div class="panel cal-card"><h2>Phase offsets</h2><div class="field"><label for="calPhaseSpeed">Speed</label>${speedSelect("calPhaseSpeed")}</div>${[1,2,3,4].map(i=>`<div class="field"><label for="calPhase${i}">Phase ${i} degrees</label><input id="calPhase${i}" type="number" min="-360" max="360" step="0.1"></div>`).join("")}<div class="button-row"><button data-cal="phase">Apply</button><button data-cal="phaseSave" class="primary">Save</button></div></div><div class="panel cal-card"><h2>Startup kick</h2><div class="field"><label for="calKickSpeed">Speed</label>${speedSelect("calKickSpeed")}</div><div class="field"><label for="calKick">Kick multiplier</label><input id="calKick" type="number" min="1" max="4" step="1"></div><div class="field"><label for="calKickDur">Kick duration sec</label><input id="calKickDur" type="number" min="0" max="15" step="1"></div><div class="field"><label for="calKickRamp">Kick ramp sec</label><input id="calKickRamp" type="number" min="0" max="15" step="0.1"></div><div class="button-row"><button data-cal="kick">Apply</button><button data-cal="kickSave" class="primary">Save</button></div></div><div class="panel cal-card"><h2>Braking</h2><div class="field"><label for="calBrakeMode">Brake mode</label><select id="calBrakeMode">${(options.brakeMode||[]).map(([v,l])=>`<option value="${v}">${esc(l)}</option>`).join("")}</select></div><div class="field"><label for="calBrakeDur">Duration sec</label><input id="calBrakeDur" type="number" min="0" max="10" step="0.1"></div><div class="field"><label for="calBrakeStart">Start frequency Hz</label><input id="calBrakeStart" type="number" min="10" max="200" step="1"></div><div class="field"><label for="calBrakeStop">Stop frequency Hz</label><input id="calBrakeStop" type="number" min="0" max="50" step="1"></div><div class="button-row"><button data-cal="brake">Apply</button><button data-cal="brakeSave" class="primary">Save</button></div></div><div class="panel cal-card"><h2>Amplitude</h2><div class="field"><label for="calAmpSpeed">Speed</label>${speedSelect("calAmpSpeed")}</div><div class="field"><label for="calMaxAmp">Global maximum amplitude percent</label><input id="calMaxAmp" type="number" min="0" max="100" step="1"></div><div class="field"><label for="calRedAmp">Reduced amplitude percent</label><input id="calRedAmp" type="number" min="10" max="100" step="1"></div><div class="button-row"><button data-cal="amp">Apply</button><button data-cal="ampSave" class="primary">Save</button></div></div>`;if(activeCalStep>4)activeCalStep=0;document.querySelectorAll("#calibrationTools .cal-card").forEach((p,i)=>p.classList.toggle("active-cal",i===activeCalStep));renderCalStepper();document.querySelectorAll("[data-cal]").forEach(b=>b.onclick=()=>applyCalibration(b.dataset.cal));fillCalibration()}
function fillCalibration(){if(!settingsData)return;const g=settingsData.global;["calSpeed","calPhaseSpeed","calKickSpeed","calAmpSpeed"].forEach(id=>{$(id).onchange=fillCalibration});const fs=Number($("calSpeed").value||0),ps=Number($("calPhaseSpeed").value||0),ks=Number($("calKickSpeed").value||0),as=Number($("calAmpSpeed").value||0);$("calFreq").value=settingsData.speeds[fs].frequency;$("calPhase1").value=settingsData.speeds[ps].phaseOffset[0];$("calPhase2").value=settingsData.speeds[ps].phaseOffset[1];$("calPhase3").value=settingsData.speeds[ps].phaseOffset[2];$("calPhase4").value=settingsData.speeds[ps].phaseOffset[3];$("calKick").value=settingsData.speeds[ks].startupKick;$("calKickDur").value=settingsData.speeds[ks].startupKickDuration;$("calKickRamp").value=settingsData.speeds[ks].startupKickRampDuration;$("calBrakeMode").value=g.brakeMode;$("calBrakeDur").value=g.brakeDuration;$("calBrakeStart").value=g.brakeStartFreq;$("calBrakeStop").value=g.brakeStopFreq;$("calMaxAmp").value=g.maxAmplitude;$("calRedAmp").value=settingsData.speeds[as].reducedAmplitude}
async function applyCalibration(kind){const save=kind.endsWith("Save");kind=kind.replace("Save","");if(kind==="frequency"){settingsData.speeds[Number($("calSpeed").value)].frequency=Number($("calFreq").value)}else if(kind==="phase"){const s=settingsData.speeds[Number($("calPhaseSpeed").value)];for(let i=0;i<4;i++)s.phaseOffset[i]=Number($(`calPhase${i+1}`).value)}else if(kind==="kick"){const s=settingsData.speeds[Number($("calKickSpeed").value)];s.startupKick=Number($("calKick").value);s.startupKickDuration=Number($("calKickDur").value);s.startupKickRampDuration=Number($("calKickRamp").value)}else if(kind==="brake"){const g=settingsData.global;g.brakeMode=Number($("calBrakeMode").value);g.brakeDuration=Number($("calBrakeDur").value);g.brakeStartFreq=Number($("calBrakeStart").value);g.brakeStopFreq=Number($("calBrakeStop").value)}else if(kind==="amp"){settingsData.global.maxAmplitude=Number($("calMaxAmp").value);settingsData.speeds[Number($("calAmpSpeed").value)].reducedAmplitude=Number($("calRedAmp").value)}renderSettings();await applySettings(save);renderCalibrationTools();addEvent(`Calibration ${kind} ${save?"saved":"applied"}`)}
function renderNetwork(){if(!networkData)return;$("networkStatus").innerHTML=`<div class="notice"><strong>${esc(networkData.status)}</strong><br>Address: ${esc(networkData.ip||"none")}<br>Mode: ${esc(networkData.modeText)}<br>SSID: ${esc(networkData.ssid||"none")}<br>Clients: ${networkData.clients}</div>`;$("networkLine").textContent=networkData.available?`${networkData.status} ${networkData.ip||""}`:"No Wi-Fi support";const root=$("networkForm");root.innerHTML="";const p=document.createElement("fieldset");p.className="panel";p.innerHTML="<legend>Network settings</legend>";for(const f of networkFields){let value=networkData.config[f.k];if(f.t==="password")value="";p.appendChild(makeField("network",f,value))}root.appendChild(p);wireForm(root,()=>{validateNetworkForm(false);updateNetworkDirty()});updateNetworkDirty();setLockedUI()}
async function loadNetwork(){networkData=await api("/api/network");deviceHomePage=Number(networkData.config.webHomePage)||0;baselineNetwork=networkComparableFromConfig(networkData.config);renderNetwork();applyDeviceHomePicker()}
function gatherNetwork(){const out={};for(const f of networkFields){const v=fieldValue("network",f);if(f.k==="password"&&v==="")continue;out[f.k]=v}return out}
async function saveNetwork(){if(!validateNetworkForm(true)){setLive("Fix network errors before saving");return}networkData=await api("/api/network",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(gatherNetwork())});baselineNetwork=networkComparableFromConfig(networkData.config);renderNetwork();await loadAuth();addEvent("Network saved");setLive("Network saved")}
async function scanNetworks(start=true){const box=$("scanResults");box.textContent="Scanning";let data=await api(`/api/network/scan${start?"?start=1":""}`);if(data.scanning){setTimeout(()=>scanNetworks(false),1200);return}box.innerHTML=`<h2>Networks</h2>${data.networks.map(n=>`<button data-ssid="${esc(n.ssid)}">${esc(n.ssid)} (${n.rssi} dBm, channel ${n.channel})</button>`).join(" ")||"No networks found"}`;box.querySelectorAll("[data-ssid]").forEach(b=>b.onclick=()=>{$(inputId("network","ssid")).value=b.dataset.ssid;updateNetworkDirty()})}
function pushTelemetry(){if(!statusData)return;const m=statusData.motor;telemetry.push({t:Date.now(),f:m.frequency,p:m.pitch,a:statusData.amp.enabled?statusData.amp.temperatureC:null,mp:m.motionProgress*100});if(telemetry.length>MAX_TELEMETRY)telemetry.shift()}
function telemetryHtml(){const a=statusData.amp.enabled?`${statusData.amp.temperatureC.toFixed(1)} C, ${statusData.amp.state}`:"not enabled";return `<div class="telemetry-grid"><div><div class="legend" aria-label="Telemetry series"><label><input type="checkbox" data-series="frequency"${telemetrySeries.frequency?" checked":""}> Frequency</label><label><input type="checkbox" data-series="pitch"${telemetrySeries.pitch?" checked":""}> Pitch</label><label><input type="checkbox" data-series="amp"${telemetrySeries.amp?" checked":""}> Amp temp</label></div><canvas class="chart" id="telemetryChart" width="720" height="240" role="img" aria-label="Live telemetry chart"></canvas></div><div id="telemetryReadout"><h3>Live telemetry</h3><p>Frequency: ${statusData.motor.frequency.toFixed(2)} Hz</p><p>Pitch: ${statusData.motor.pitch.toFixed(2)} percent</p><p>Motion progress: ${Math.round(statusData.motor.motionProgress*100)} percent</p><p>Amplifier: ${esc(a)}</p></div></div>`}
function drawSeries(ctx,vals,color,min,max){if(vals.length<2)return;const w=720,h=220,pad=28,range=Math.max(max-min,0.001);ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();vals.forEach((v,i)=>{const x=pad+i*(w-pad*2)/(vals.length-1),y=h-pad-((v-min)/range)*(h-pad*2);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke()}
function drawTelemetry(){const c=$("telemetryChart");if(!c||!telemetry.length)return;document.querySelectorAll("[data-series]").forEach(el=>el.onchange=()=>{telemetrySeries[el.dataset.series]=el.checked;drawTelemetry()});const ctx=c.getContext("2d"),style=getComputedStyle(document.documentElement),w=720,h=240,pad=32;ctx.clearRect(0,0,w,h);ctx.strokeStyle=style.getPropertyValue("--line");ctx.lineWidth=1;for(let i=0;i<=4;i++){const y=20+i*42;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w-pad,y);ctx.stroke()}for(let i=0;i<=6;i++){const x=pad+i*(w-pad*2)/6;ctx.beginPath();ctx.moveTo(x,20);ctx.lineTo(x,188);ctx.stroke()}ctx.strokeRect(pad,20,w-pad*2,168);ctx.fillStyle=style.getPropertyValue("--muted");ctx.fillText("newer",w-pad-38,214);ctx.fillText("older",pad,214);const f=telemetry.map(x=>x.f),p=telemetry.map(x=>x.p),a=telemetry.filter(x=>x.a!==null).map(x=>x.a);if(telemetrySeries.frequency)drawSeries(ctx,f,style.getPropertyValue("--accent"),Math.min(...f),Math.max(...f));if(telemetrySeries.pitch)drawSeries(ctx,p,style.getPropertyValue("--accent2"),-50,50);if(telemetrySeries.amp&&a.length)drawSeries(ctx,a,style.getPropertyValue("--warn"),20,90);let lx=pad;[["frequency","Frequency","--accent"],["pitch","Pitch","--accent2"],["amp","Amp temp","--warn"]].forEach(([k,l,cvar])=>{if(!telemetrySeries[k])return;ctx.fillStyle=style.getPropertyValue(cvar);ctx.fillRect(lx,222,12,4);ctx.fillStyle=style.getPropertyValue("--muted");ctx.fillText(l,lx+16,228);lx+=95})}
function renderDashboard(){if(!statusData)return;const b=$("dashboardBody"),m=statusData.motor,amp=statusData.amp.enabled?`${statusData.amp.temperatureC.toFixed(1)} C`:"Off",cls=statusClass(m.state);const tiles=`<div class="dash-grid"><div class="dash-tile"><span>State</span><strong><span class="status-chip ${cls}">${m.state}</span></strong></div><div class="dash-tile"><span>Speed</span><strong>${speedNames[m.speed]}</strong></div><div class="dash-tile"><span>Frequency</span><strong>${m.frequency.toFixed(2)} Hz</strong></div><div class="dash-tile"><span>Pitch</span><strong>${m.pitch.toFixed(2)}%</strong></div><div class="dash-tile"><span>Motion</span><strong>${Math.round(m.motionProgress*100)}%</strong><div class="progress"><i style="width:${Math.round(m.motionProgress*100)}%"></i></div></div><div class="dash-tile"><span>Amplifier</span><strong>${amp}</strong></div></div>`;if(dashboardMode==="stats"){b.innerHTML=`<h2>Runtime</h2>${tiles}<div class="dash-grid"><div class="dash-tile"><span>Session</span><strong>${statusData.runtime.session}s</strong></div><div class="dash-tile"><span>Total</span><strong>${statusData.runtime.total}s</strong></div></div>${telemetryHtml()}`;drawTelemetry();return}if(dashboardMode==="dim"){b.innerHTML=`<div class="dash-tile"><span>${m.state}</span><strong>${speedNames[m.speed]}</strong><div class="progress"><i style="width:${Math.round(m.motionProgress*100)}%"></i></div></div>`;return}if(dashboardMode==="scope"){b.innerHTML=`<h2>Scope</h2>${tiles}<canvas class="scope" id="scopeCanvas" width="180" height="180" role="img" aria-label="Phase scope"></canvas><p>Phase A: ${statusData.scope.a}, Phase B: ${statusData.scope.b}</p>${telemetryHtml()}`;const c=$("scopeCanvas"),x=c.getContext("2d");x.clearRect(0,0,180,180);x.strokeStyle=getComputedStyle(document.documentElement).getPropertyValue("--line");x.strokeRect(20,20,140,140);x.fillStyle=getComputedStyle(document.documentElement).getPropertyValue("--accent");x.beginPath();x.arc(90+statusData.scope.a/8,90-statusData.scope.b/8,5,0,Math.PI*2);x.fill();drawTelemetry();return}b.innerHTML=`<h2>${speedNames[m.speed]}</h2>${tiles}${telemetryHtml()}`;drawTelemetry()}
function renderStatus(){if(!statusData)return;const m=statusData.motor,ampText=statusData.amp.enabled?(statusData.amp.thermalOk?"OK":"TRIPPED"):"Off";if(lastState&&lastState!==m.state)addEvent(`Motor state ${m.state}`);if(lastSpeed>=0&&lastSpeed!==m.speed)addEvent(`Speed ${speedNames[m.speed]||m.speedName}`);if(lastAmpState&&lastAmpState!==ampText)addEvent(`Amplifier ${ampText}`);lastState=m.state;lastSpeed=m.speed;lastAmpState=ampText;$("state").textContent=m.state;$("speed").textContent=speedNames[m.speed]||m.speedName||"-";$("frequency").textContent=m.frequency.toFixed(2)+" Hz";$("pitch").textContent=m.pitch.toFixed(2)+"%";$("ampState").textContent=ampText;const cls=statusClass(m.state);$("state").closest(".metric").className=`metric ${cls}`;$("speed").closest(".metric").className=`metric ${m.running?"ok":"warn"}`;$("frequency").closest(".metric").className=`metric ${m.speedRamping?"warn":m.running?"ok":""}`;$("pitch").closest(".metric").className=`metric ${Math.abs(m.pitch)>0.01?"warn":""}`;$("ampState").closest(".metric").className=`metric ${ampText==="TRIPPED"?"bad":ampText==="OK"?"ok":""}`;if(document.activeElement!==$("pitchControl"))$("pitchControl").value=m.pitch.toFixed(1);if($("simplePitch")&&document.activeElement!==$("simplePitch"))$("simplePitch").value=m.pitch.toFixed(1);const relay=$("relayStage");if(relay.children.length!==m.relayStageCount){relay.innerHTML="";for(let i=0;i<m.relayStageCount;i++){const o=document.createElement("option");o.value=i;o.textContent=`Stage ${i}`;relay.appendChild(o)}}relay.value=String(m.relayStage);pushTelemetry();renderDashboard();renderBench()}
async function loadStatus(){statusData=await api("/api/status");renderStatus()}
function startStatusStream(){if(!("EventSource" in window)){setInterval(loadStatus,1000);return}let fallback=false;const es=new EventSource("/api/events");es.addEventListener("status",e=>{try{statusData=JSON.parse(e.data);renderStatus()}catch(err){}});es.onerror=()=>{if(!fallback&&!telemetry.length){fallback=true;es.close();setInterval(loadStatus,1000)}}}
async function setSpeedControl(speed){if(Number(speed)===2&&!is78Enabled()){const msg=disabled78Message();alert(msg);setLive(msg);return}await control("setSpeed",{speed:Number(speed)})}
async function control(action,extra={}){await api("/api/control",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(Object.assign({action},extra))});addEvent(`Command ${action}`);await loadStatus()}
function currentPresetShape(){const g=settingsComparable(currentSettings(false)).global;return{pm:g.phaseMode,maxAmp:g.maxAmplitude,ssCurve:g.softStartCurve,fda:g.freqDependentAmplitude,vfLF:g.vfLowFreq,vfLB:g.vfLowBoost,vfMF:g.vfMidFreq,vfMB:g.vfMidBoost,brkMd:g.brakeMode,brkDur:g.brakeDuration,brkPG:g.brakePulseGap,brkSF:g.brakeStartFreq,brkStF:g.brakeStopFreq,brkCut:g.softStopCutoff,speeds:settingsData.speeds.map(s=>({f:s.frequency,minF:s.minFrequency,maxF:s.maxFrequency,ph:[...s.phaseOffset],ssD:s.softStartDuration,rAmp:s.reducedAmplitude,aDly:s.amplitudeDelay,kick:s.startupKick,kDur:s.startupKickDuration,kRmp:s.startupKickRampDuration,fTyp:s.filterType,iir:s.iirAlpha,fir:s.firProfile}))}}
function presetPathLabel(path){const names={pm:"Phase mode",maxAmp:"Maximum amplitude",ssCurve:"Soft start curve",fda:"V/f blend",vfLF:"V/f low frequency",vfLB:"V/f low boost",vfMF:"V/f mid frequency",vfMB:"V/f mid boost",brkMd:"Brake mode",brkDur:"Brake duration",brkPG:"Brake pulse gap",brkSF:"Brake start frequency",brkStF:"Brake stop frequency",brkCut:"Soft stop cutoff",f:"Frequency",minF:"Minimum frequency",maxF:"Maximum frequency",ssD:"Soft start duration",rAmp:"Reduced amplitude",aDly:"Amplitude delay",kick:"Startup kick",kDur:"Startup kick duration",kRmp:"Startup kick ramp",fTyp:"Filter type",iir:"IIR alpha",fir:"FIR profile"};const p=path.split(".");if(p[0]==="speeds"){if(p[2]==="ph")return `${speedNames[Number(p[1])]} phase ${Number(p[3])+1} offset`;return `${speedNames[Number(p[1])]} ${names[p[2]]||p[2]}`}return names[path]||path}
function renderPresetDiff(slot,title,d,report=null){const box=$(`presetPreview${slot}`);if(!box)return;box.classList.remove("hide");const diffText=d&&d.length?d.slice(0,36).map(x=>`${presetPathLabel(x.path)}: ${displayValue(x.path,x.from)} -> ${displayValue(x.path,x.to)}`).join("\n")+(d.length>36?`\n${d.length-36} more changes.`:""):"No differences from current motor settings.";if(report){renderReport(box,title,report,`<h4>Previewed changes</h4><pre>${esc(diffText)}</pre>`);return}box.textContent=`${title}\n${diffText}`}
function mergePresetShape(base,patch){const out=clone(base);function merge(a,b){Object.keys(b||{}).forEach(k=>{if(b[k]&&typeof b[k]==="object"&&!Array.isArray(b[k])){a[k]=a[k]||{};merge(a[k],b[k])}else a[k]=b[k]})}merge(out,patch);return out}
async function previewPreset(slot,sourceText=null,title="Preset load preview"){const box=$(`presetPreview${slot}`);let json=sourceText;if(!json){try{const res=await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot,action:"export"})});json=res.json}catch(e){if(box){box.classList.remove("hide");box.textContent=e.message}setLive(e.message);return null}}let parsed;try{parsed=JSON.parse(json)}catch(e){if(box){box.classList.remove("hide");box.innerHTML=`<h3>${esc(title)}</h3><div class="report-item report-error"><strong>Error:</strong> Preset JSON is not valid.</div>`}return null}const report=validatePresetImportObject(parsed),current=currentPresetShape(),target=sourceText?mergePresetShape(current,parsed):parsed,d=diffs(current,target);renderPresetDiff(slot,title,d,report);return{diff:d,report}}
async function loadPresets(){presetsData=await api("/api/presets");fillPresetCompare();const root=$("presetGrid");root.innerHTML="";presetsData.presets.forEach(p=>{const card=document.createElement("div");card.className="preset-card";card.innerHTML=`<h2>Slot ${p.slot+1}</h2><p>${p.stored?"Stored preset":"Empty slot"}</p><div class="field"><label for="presetName${p.slot}">Name</label><input id="presetName${p.slot}" type="text" value="${esc(p.name)}"></div><div class="button-row"><button data-pa="preview">Preview load</button><button data-pa="load">Load</button><button data-pa="save">Save current</button><button data-pa="rename">Rename</button><button data-pa="export">Export</button><button data-pa="previewImport">Preview import</button><button data-pa="import">Import</button><button class="danger" data-pa="clear">Clear</button></div><textarea id="presetText${p.slot}" rows="7" aria-label="Preset JSON"></textarea><div id="presetPreview${p.slot}" class="diff hide" aria-live="polite"></div>`;card.querySelectorAll("[data-pa]").forEach(b=>b.onclick=()=>presetAction(p.slot,b.dataset.pa));root.appendChild(card)});setLockedUI()}
async function presetAction(slot,action){let body={slot,action};if(action==="preview"){await previewPreset(slot);return}if(action==="previewImport"){await previewPreset(slot,$(`presetText${slot}`).value,"Import validation report");return}if(action==="load"){const r=await previewPreset(slot);if(!r||r.report.errors||!confirm("Load this preset into the live settings?"))return}if(action==="rename")body.name=$(`presetName${slot}`).value;if(action==="import"){const r=await previewPreset(slot,$(`presetText${slot}`).value,"Import validation report");if(!r||r.report.errors){setLive("Fix preset import errors before importing");return}if(!confirm(`Import this JSON into the preset slot?\n\n${r.report.warnings} warning${r.report.warnings===1?"":"s"} will be accepted.`))return;body.json=$(`presetText${slot}`).value}if(action==="clear"&&!confirm("Clear this preset?"))return;const res=await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});if(action==="export")$(`presetText${slot}`).value=res.json||"";await loadPresets();await loadSettings();setLive("Preset action complete")}
function fillPresetCompare(){["compareA","compareB"].forEach(id=>{const el=$(id);if(!el||el.children.length)return;for(let i=0;i<5;i++){const o=document.createElement("option");o.value=i;o.textContent=`Slot ${i+1}`;el.appendChild(o)}});if($("compareB"))$("compareB").value="1"}
async function comparePresetSlots(){const a=Number($("compareA").value),b=Number($("compareB").value),box=$("presetCompare");try{const ja=JSON.parse((await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot:a,action:"export"})})).json),jb=JSON.parse((await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot:b,action:"export"})})).json),d=diffs(ja,jb);box.classList.remove("hide");box.textContent=d.length?d.map(x=>`${presetPathLabel(x.path)}: ${displayValue(x.path,x.from)} -> ${displayValue(x.path,x.to)}`).join("\n"):"Preset slots match."}catch(e){box.classList.remove("hide");box.textContent=e.message}}
async function loadDiagnostics(){const d=await api("/api/diagnostics");diagnosticsData=d;const body=$("diagnosticsBody");body.innerHTML=`<h2>Diagnostics</h2><div class="tool-grid"><div><h3>Build</h3><p>Firmware: ${esc(d.firmware)}</p><p>Build: ${esc(d.buildDate)}</p><p>Safe mode: ${d.safeMode?"yes":"no"}</p></div><div><h3>Network</h3><p>${esc(d.network.status)} ${esc(d.network.ip||"")}</p><p>RSSI: ${d.network.rssi} dBm</p><p>Clients: ${d.network.clients}</p><p>Read-only mode: ${d.network.readOnlyMode?"on":"off"}</p></div><div><h3>Amplifier</h3><p>${d.amp.enabled?`${Number(d.amp.temperatureC).toFixed(1)} C, thermal ${d.amp.thermalOk?"OK":"TRIPPED"}, warn ${Number(d.amp.warnC).toFixed(0)} C, shutdown ${Number(d.amp.shutdownC).toFixed(0)} C`:"not enabled"}</p></div></div><h3>Feature flags</h3><div class="log">${Object.keys(d.flags).map(k=>`${k}: ${d.flags[k]}`).join("\n")}</div><h3>Pins</h3><div class="log">${Object.keys(d.pins).map(k=>`${k}: GP${d.pins[k]}`).join("\n")}</div><h3>Files</h3><div class="log">settings: ${d.files.settings}\nnetwork: ${d.files.network}\nerrors: ${d.files.errors}\npresets: ${d.files.presets.map(p=>`slot ${p.slot+1}=${p.stored}`).join(", ")}</div>`;renderEventFeed();renderBench()}
function relayStageOptions(){const count=statusData?.motor?.relayStageCount||1;let out="";for(let i=0;i<count;i++)out+=`<option value="${i}">Stage ${i}</option>`;return out}
function benchReportText(){const m=statusData?.motor||{},n=statusData?.network||{},a=statusData?.amp||{},d=diagnosticsData;return[`TT Control bench report`,new Date().toISOString(),`State: ${m.state||"-"}`,`Speed: ${speedNames[m.speed]||m.speedName||"-"}`,`Frequency: ${m.frequency!==undefined?m.frequency.toFixed(2):"-"} Hz`,`Pitch: ${m.pitch!==undefined?m.pitch.toFixed(2):"-"}%`,`Relay test: ${m.relayTest?"on":"off"} stage ${m.relayStage??"-"}`,`Amp: ${a.enabled?(Number(a.temperatureC).toFixed(1)+" C, thermal "+(a.thermalOk?"OK":"TRIPPED")):"not enabled"}`,`Network: ${n.status||"-"} ${n.ip||""}`,d?`Firmware: ${d.firmware} ${d.buildDate}`:"Firmware: not loaded",d?`Pins: ${Object.keys(d.pins).map(k=>`${k}=GP${d.pins[k]}`).join(", ")}`:"Pins: not loaded"].join("\n")}
function renderBench(){const root=$("benchBody");if(!root||currentTab!=="bench")return;if(root.contains(document.activeElement)&&["INPUT","SELECT","TEXTAREA"].includes(document.activeElement.tagName))return;const m=statusData?.motor||{},a=statusData?.amp||{},ampText=a.enabled?`${Number(a.temperatureC).toFixed(1)} C, ${a.thermalOk?"OK":"TRIPPED"}`:"not enabled";root.innerHTML=`<div class="panel section-head"><h2>Bench test</h2><div class="dash-grid"><div class="dash-tile"><span>Motor state</span><strong>${esc(m.state||"-")}</strong></div><div class="dash-tile"><span>Relay test</span><strong>${m.relayTest?"On":"Off"}</strong></div><div class="dash-tile"><span>Amplifier</span><strong>${esc(ampText)}</strong></div></div></div><div class="bench-grid"><div class="bench-card"><h3>Pre-check</h3><div class="button-row"><button id="benchRefresh">Refresh diagnostics</button><button class="danger" data-bench="emergencyStop">Emergency stop</button><button data-bench="stop">Stop</button></div><p>Safe mode: ${diagnosticsData?.safeMode?"yes":"no"}</p><p>Network: ${esc(statusData?.network?.status||"-")} ${esc(statusData?.network?.ip||"")}</p></div><div class="bench-card"><h3>Relay outputs</h3><div class="field"><label for="benchRelayStage">Relay output</label><select id="benchRelayStage">${relayStageOptions()}</select></div><div class="button-row"><button data-bench="relayTest">Set output</button><button data-bench="relayOff">All off</button></div></div><div class="bench-card"><h3>Brake test</h3><div class="button-row"><button class="good" data-bench="start">Start motor</button><button class="danger" data-bench="stop">Brake stop</button><button class="danger" data-bench="emergencyStop">Emergency stop</button></div></div><div class="bench-card"><h3>Speed and pitch</h3><div class="button-row"><button data-bench-speed="0">33 RPM</button><button data-bench-speed="1">45 RPM</button><button data-bench-speed="2">78 RPM</button><button data-bench="resetPitch">Reset pitch</button></div><div class="field"><label for="benchPitch">Pitch percent</label><input id="benchPitch" type="number" min="-50" max="50" step="0.1" value="${m.pitch!==undefined?Number(m.pitch).toFixed(1):"0"}"></div><button id="benchSetPitch">Set pitch</button></div><div class="bench-card"><h3>Report</h3><div class="button-row"><button id="benchMakeReport">Generate report</button></div><textarea id="benchReport" aria-label="Bench test report">${esc(benchReportText())}</textarea></div></div>`;$("benchRelayStage").value=String(m.relayStage||0);$("benchRefresh").onclick=()=>loadDiagnostics().catch(e=>setLive(e.message));$("benchSetPitch").onclick=()=>control("setPitch",{pitch:Number($("benchPitch").value)}).catch(e=>setLive(e.message));$("benchMakeReport").onclick=()=>{$("benchReport").value=benchReportText()};document.querySelectorAll("[data-bench]").forEach(b=>b.onclick=()=>{const action=b.dataset.bench;if((action==="start"||action==="relayTest")&&!confirm("Run this bench test action now?"))return;const extra=action==="relayTest"?{stage:Number($("benchRelayStage").value)}:{};control(action,extra).catch(e=>setLive(e.message))});document.querySelectorAll("[data-bench-speed]").forEach(b=>b.onclick=()=>setSpeedControl(b.dataset.benchSpeed).catch(e=>setLive(e.message)));setLockedUI()}
async function loadBench(){if(!diagnosticsData)await loadDiagnostics();else renderBench()}
async function exportBackup(){const backup={format:"TTControl backup",version:1,created:new Date().toISOString(),settings:await api("/api/settings"),network:await api("/api/network"),presets:[],errors:await api("/api/errors")};delete backup.network.config.password;delete backup.network.config.apPassword;delete backup.network.config.webPin;const p=await api("/api/presets");for(const slot of p.presets){if(slot.stored){try{const ex=await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot:slot.slot,action:"export"})});backup.presets.push({slot:slot.slot,name:slot.name,json:ex.json})}catch(e){}}}$("backupText").value=JSON.stringify(backup,null,2);addEvent("Backup exported")}
function validateBackupText(){let b;const box=$("backupReport");try{b=JSON.parse($("backupText").value)}catch(e){const r=newReport();reportIssue(r,"error","backup","Backup JSON is not valid.");renderReport(box,"Import validation report",r);return null}const report=validateBackupObject(b);renderReport(box,"Import validation report",report);return{backup:b,report}}
async function importBackup(){const checked=validateBackupText();if(!checked)return;const b=checked.backup,r=checked.report;if(r.errors){setLive("Fix backup import errors before importing");return}if(!confirm(`Import backup settings and presets?\n\n${r.warnings} warning${r.warnings===1?"":"s"} will be accepted. Network passwords are not included in backups.`))return;if(b.settings)await api("/api/settings",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(Object.assign({},b.settings,{save:true}))});if(b.network&&b.network.config){const n=Object.assign({},b.network.config);delete n.password;delete n.apPassword;delete n.webPin;await api("/api/network",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(n)})}if(Array.isArray(b.presets)){for(const p of b.presets){if(p.json)await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot:p.slot,action:"import",json:p.json})});if(p.name)await api("/api/preset",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({slot:p.slot,action:"rename",name:p.name})})}}await Promise.all([loadSettings(),loadNetwork(),loadPresets(),loadDiagnostics()]);addEvent("Backup imported");setLive("Backup imported")}
async function loadErrors(){const e=await api("/api/errors");$("errorLog").textContent=e.lines.join("\n")||"No errors"}
function setLarge(enabled){document.body.classList.toggle("large",enabled);try{localStorage.setItem("ttc_large",enabled?"1":"0")}catch(e){}}
function applyTheme(theme){if(!theme)theme="system";document.body.dataset.theme=theme;try{localStorage.setItem("ttc_theme",theme)}catch(e){}const el=$("themeSelect");if(el)el.value=theme}
function deviceHomeTab(){return homeTabs[deviceHomePage]||"dashboard"}
function applyDeviceHomePicker(){const home=$("homeSelect");if(!home)return;home.value=String(homeTabs[deviceHomePage]?deviceHomePage:0);home.disabled=false;setLockedUI()}
async function saveHomePreference(){const home=$("homeSelect");if(!home)return;const previous=deviceHomePage,value=Number(home.value);try{const pref=await api("/api/preferences",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({homePage:value})});deviceHomePage=Number(pref.homePage)||0;if(networkData&&networkData.config){networkData.config.webHomePage=deviceHomePage;baselineNetwork=networkComparableFromConfig(networkData.config);updateNetworkDirty()}applyDeviceHomePicker();setLive(`Home page saved on device: ${tabNames[deviceHomeTab()]}`)}catch(e){deviceHomePage=previous;home.value=String(previous);setLive(e.message)}}
function initPrefs(){try{document.body.classList.toggle("large",localStorage.getItem("ttc_large")==="1");const legacy=localStorage.getItem("ttc_contrast")==="1"&&!localStorage.getItem("ttc_theme");applyTheme(legacy?"contrast":localStorage.getItem("ttc_theme")||"system")}catch(e){applyTheme("system")}const home=$("homeSelect");if(home){home.innerHTML=homeTabs.map((id,i)=>`<option value="${i}">${tabNames[id]}</option>`).join("");home.value="0";home.disabled=true;home.onchange=()=>saveHomePreference().catch(e=>setLive(e.message))}$("themeSelect").onchange=()=>applyTheme($("themeSelect").value);$("largeToggle").onclick=()=>setLarge(!document.body.classList.contains("large"))}
function loadTabContent(tab){if(tab==="network")loadNetwork().catch(e=>setLive(e.message));if(tab==="presets")loadPresets().catch(e=>setLive(e.message));if(tab==="bench")loadBench().catch(e=>setLive(e.message));if(tab==="diagnostics")loadDiagnostics().catch(e=>setLive(e.message));if(tab==="errors")loadErrors().catch(e=>setLive(e.message));if(tab==="calibrate")renderCalibrationTools()}
function activateTab(tab,focus=true){if(!$(tab))tab="dashboard";currentTab=tab;document.querySelectorAll("[data-tab]").forEach(x=>x.setAttribute("aria-selected",String(x.dataset.tab===tab)));document.querySelectorAll("main section").forEach(s=>{s.classList.toggle("active",s.id===tab);s.tabIndex=-1});if(focus)$(tab).focus();loadTabContent(tab)}
function wire(){initPrefs();try{authToken=sessionStorage.getItem("ttc_token")||""}catch(e){}document.querySelectorAll("[data-tab]").forEach(b=>b.onclick=()=>activateTab(b.dataset.tab));document.querySelectorAll("[data-mode]").forEach(b=>b.onclick=()=>{dashboardMode=b.dataset.mode;document.querySelectorAll("[data-mode]").forEach(x=>x.setAttribute("aria-pressed",String(x===b)));renderDashboard()});document.querySelectorAll("[data-action]").forEach(b=>b.onclick=()=>control(b.dataset.action).catch(e=>setLive(e.message)));document.querySelectorAll("[data-speed]").forEach(b=>b.onclick=()=>setSpeedControl(b.dataset.speed).catch(e=>setLive(e.message)));document.querySelectorAll("[data-speed-tab]").forEach(b=>b.onclick=()=>{const next=Number(b.dataset.speedTab);if(next===2&&!is78Enabled()){const msg=disabled78Message();alert(msg);setLive(msg);return}captureSpeedTab();speedTab=next;renderSpeedSettings()});$("unlockWeb").onclick=()=>unlockWeb().catch(e=>setLive(e.message));$("lockWeb").onclick=()=>lockWeb().catch(e=>setLive(e.message));$("setPitch").onclick=()=>control("setPitch",{pitch:Number($("pitchControl").value)}).catch(e=>setLive(e.message));$("setSimplePitch").onclick=()=>control("setPitch",{pitch:Number($("simplePitch").value)}).catch(e=>setLive(e.message));$("relayTest").onclick=()=>control("relayTest",{stage:Number($("relayStage").value)}).catch(e=>setLive(e.message));$("settingsSearch").oninput=filterSettings;$("reviewSettings").onclick=reviewSettingsChanges;$("applySave").onclick=()=>applySettings(true).catch(e=>setLive(e.message));$("applyOnly").onclick=()=>applySettings(false).catch(e=>setLive(e.message));$("discardSettings").onclick=loadSettings;$("factoryReset").onclick=()=>{if(confirm("Factory reset all settings and presets?"))control("factoryReset").then(()=>Promise.all([loadSettings(),loadNetwork()])).catch(e=>setLive(e.message))};$("resetRuntime").onclick=()=>control("resetRuntime").catch(e=>setLive(e.message));$("saveNetwork").onclick=()=>saveNetwork().catch(e=>setLive(e.message));$("discardNetwork").onclick=loadNetwork;$("scanNetworks").onclick=()=>scanNetworks(true).catch(e=>setLive(e.message));$("comparePresets").onclick=()=>comparePresetSlots().catch(e=>setLive(e.message));$("exportBackup").onclick=()=>exportBackup().catch(e=>setLive(e.message));$("validateBackup").onclick=()=>validateBackupText();$("importBackup").onclick=()=>importBackup().catch(e=>setLive(e.message));$("clearErrors").onclick=()=>api("/api/errors",{method:"POST"}).then(()=>{addEvent("Error log cleared");loadErrors()}).catch(e=>setLive(e.message))}
async function start(){wire();await loadSchema();await Promise.all([loadAuth(),loadStatus(),loadSettings(),loadNetwork()]);activateTab(deviceHomeTab(),false);startStatusStream();setInterval(loadAuth,10000)}
start().catch(e=>setLive(e.message));
</script>
</body>
</html>
)HTML";

static const char SETUP_HTML[] = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TT Control Wi-Fi Setup</title>
<style>
:root{color-scheme:light dark;--bg:#f7f7f4;--panel:#fff;--text:#161714;--muted:#5d625a;--line:#d8dbd2;--accent:#2364aa;--danger:#b3261e;--focus:#ffbf47}
@media (prefers-color-scheme:dark){:root{--bg:#111412;--panel:#1b1f1c;--text:#f2f4ef;--muted:#b8beb4;--line:#343a35;--accent:#79aef2;--danger:#ffb4ab;--focus:#ffd166}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:18px/1.45 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}.wrap{max-width:820px;margin:0 auto;padding:1rem}header,.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:1rem;margin:1rem 0}h1{font-size:1.6rem;margin:.2rem 0}.muted{color:var(--muted)}label{display:block;font-weight:700;margin:.8rem 0 .25rem}input,select,button{font:inherit;border:1px solid var(--line);border-radius:6px;background:var(--panel);color:var(--text)}input,select{width:100%;min-height:50px;padding:.45rem .6rem}button{min-height:50px;padding:.55rem .9rem;margin:.25rem .25rem .25rem 0;cursor:pointer}button.primary{background:var(--accent);border-color:var(--accent);color:#fff}button.danger{background:var(--danger);border-color:var(--danger);color:#fff}button:focus-visible,input:focus-visible,select:focus-visible{outline:4px solid var(--focus);outline-offset:2px}.check{display:flex;align-items:center;gap:.6rem}.check input{width:1.2rem}.networks button{display:block;width:100%;text-align:left}.notice{border-left:4px solid var(--accent)}.steps{display:grid;grid-template-columns:repeat(3,1fr);gap:.4rem}.steps button[aria-current=true]{border-color:var(--accent);box-shadow:inset 0 0 0 2px var(--accent)}.field-error{color:var(--danger);font-weight:700;min-height:1.2em}.row{display:grid;grid-template-columns:1fr 1fr;gap:.75rem}.button-row{display:flex;flex-wrap:wrap;gap:.5rem}.summary{white-space:pre-wrap;background:var(--bg);border:1px solid var(--line);border-radius:8px;padding:.75rem}@media(max-width:700px){.steps,.row{grid-template-columns:1fr}}
</style>
</head>
<body>
<div class="wrap">
<header>
<h1>TT Control Wi-Fi Setup</h1>
<p class="muted">This open setup network only allows network configuration. Motor controls and device settings are blocked until the device is reached through your configured Wi-Fi network.</p>
</header>
<section class="panel notice" id="status" aria-live="polite">Loading network status</section>
<nav class="steps" aria-label="Setup steps"><button data-step="0" aria-current="true">1. Network</button><button data-step="1" aria-current="false">2. Address</button><button data-step="2" aria-current="false">3. Setup AP</button></nav>
<section class="panel" id="step0" tabindex="-1">
<h2>Network</h2>
<label class="check"><input id="enabled" type="checkbox" checked> Wi-Fi enabled</label>
<label for="mode">Connection mode</label><select id="mode"><option value="0">Setup AP only</option><option value="1">Connect to Wi-Fi</option><option value="2">Connect to Wi-Fi and keep setup AP</option></select>
<label for="hostname">Hostname</label><input id="hostname" type="text">
<label for="ssid">Wi-Fi network name</label><input id="ssid" type="text" autocomplete="off"><div id="ssidErr" class="field-error" aria-live="polite"></div>
<label for="password">Wi-Fi password</label><input id="password" type="password" autocomplete="new-password">
<button id="scan">Scan Wi-Fi networks</button>
</section>
<section class="panel" id="step1" tabindex="-1" hidden>
<h2>Address</h2>
<label class="check"><input id="dhcp" type="checkbox" checked> Use DHCP</label>
<div class="row"><div><label for="staticIp">Static IP</label><input id="staticIp" type="text" inputmode="numeric"><div id="staticIpErr" class="field-error" aria-live="polite"></div></div><div><label for="gateway">Gateway</label><input id="gateway" type="text" inputmode="numeric"><div id="gatewayErr" class="field-error" aria-live="polite"></div></div></div>
<div class="row"><div><label for="subnet">Subnet mask</label><input id="subnet" type="text" inputmode="numeric"><div id="subnetErr" class="field-error" aria-live="polite"></div></div><div><label for="dns">DNS server</label><input id="dns" type="text" inputmode="numeric"><div id="dnsErr" class="field-error" aria-live="polite"></div></div></div>
</section>
<section class="panel" id="step2" tabindex="-1" hidden>
<h2>Setup AP</h2>
<label class="check"><input id="apFallback" type="checkbox" checked> Reopen setup network if Wi-Fi connection fails</label>
<label for="apSsid">Setup network name</label><input id="apSsid" type="text"><div id="apSsidErr" class="field-error" aria-live="polite"></div>
<label for="apPassword">Setup network password</label><input id="apPassword" type="password" autocomplete="new-password" aria-describedby="apHelp"><p id="apHelp" class="muted">Leave blank for an open setup network. Open setup mode only allows Wi-Fi configuration.</p><div id="apPasswordErr" class="field-error" aria-live="polite"></div>
<label for="apChannel">Setup channel</label><input id="apChannel" type="number" min="1" max="13" step="1"><div id="apChannelErr" class="field-error" aria-live="polite"></div>
<h3>Summary</h3><div id="summary" class="summary"></div>
</section>
<div class="panel button-row"><button id="back">Back</button><button id="next" class="primary">Next</button><button class="primary" id="save" hidden>Save and reconnect</button></div>
<section class="panel networks" id="networks"></section>
</div>
<script>
const $=id=>document.getElementById(id);
function esc(v){return String(v??"").replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]))}
async function api(path,opts){const r=await fetch(path,opts);const t=await r.text();let d={};try{d=t?JSON.parse(t):{}}catch(e){throw new Error(t||r.statusText)}if(!r.ok)throw new Error(d.error||r.statusText);return d}
let step=0;
function setStatus(n){$("status").innerHTML=`<strong>${esc(n.status)}</strong><br>Address: ${esc(n.ip||"none")}<br>SSID: ${esc(n.ssid||"none")}`}
function err(id,msg){$(id).textContent=msg||"";return !msg}
function validIp(v){return /^(\d{1,3}\.){3}\d{1,3}$/.test(v)&&v.split(".").every(x=>Number(x)>=0&&Number(x)<=255)}
function updateSummary(){if(!$("summary"))return;$("summary").textContent=`Mode: ${$("mode").selectedOptions[0]?.textContent||""}\nHostname: ${$("hostname").value||"ttcontrol"}\nWi-Fi: ${$("ssid").value||"not set"}\nAddress: ${$("dhcp").checked?"DHCP":$("staticIp").value}\nSetup AP: ${$("apSsid").value||"TTControl-Setup"}\nSetup AP security: ${$("apPassword").value?"password protected":"open"}`}
function showStep(n){step=Math.max(0,Math.min(2,n));[0,1,2].forEach(i=>{$(`step${i}`).hidden=i!==step;document.querySelector(`[data-step="${i}"]`).setAttribute("aria-current",String(i===step))});$("back").disabled=step===0;$("next").hidden=step===2;$("save").hidden=step!==2;updateSummary();$(`step${step}`).focus()}
function validate(n=step){let ok=true;err("ssidErr","");err("apSsidErr","");err("apPasswordErr","");err("apChannelErr","");["staticIp","gateway","subnet","dns"].forEach(k=>err(`${k}Err`,""));if(n===0&&Number($("mode").value)!==0&&!$("ssid").value.trim())ok=err("ssidErr","Wi-Fi network name is required for station mode.")&&ok;if(n===1&&!$("dhcp").checked){["staticIp","gateway","subnet","dns"].forEach(k=>{if(!validIp($(k).value.trim()))ok=err(`${k}Err`,"Enter a valid IPv4 address.")&&ok})}if(n===2){if(!$("apSsid").value.trim())ok=err("apSsidErr","Setup network name is required.")&&ok;if($("apPassword").value&&$("apPassword").value.length<8)ok=err("apPasswordErr","Use at least 8 characters, or leave blank for open setup.")&&ok;const ch=Number($("apChannel").value);if(ch<1||ch>13)ok=err("apChannelErr","Choose channel 1 through 13.")&&ok}return ok}
async function load(){const n=await api("/api/network");setStatus(n);const c=n.config;$("enabled").checked=c.enabled;$("mode").value=c.mode;$("hostname").value=c.hostname;$("ssid").value=c.ssid;$("password").value="";$("dhcp").checked=c.dhcp;$("staticIp").value=c.staticIp;$("gateway").value=c.gateway;$("subnet").value=c.subnet;$("dns").value=c.dns;$("apFallback").checked=c.apFallback;$("apSsid").value=c.apSsid;$("apPassword").value="";$("apChannel").value=c.apChannel;updateSummary()}
function body(){const out={enabled:$("enabled").checked,mode:Number($("mode").value),hostname:$("hostname").value,ssid:$("ssid").value,dhcp:$("dhcp").checked,staticIp:$("staticIp").value,gateway:$("gateway").value,subnet:$("subnet").value,dns:$("dns").value,apFallback:$("apFallback").checked,apSsid:$("apSsid").value,apPassword:$("apPassword").value,apChannel:Number($("apChannel").value)};if($("password").value!=="")out.password=$("password").value;return out}
async function save(){if(![0,1,2].every(i=>validate(i)))return;const n=await api("/api/network",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body())});setStatus(n);$("status").innerHTML+="<br>Saved. If the setup network closes, reconnect through the configured Wi-Fi network."}
async function scan(start=true){$("networks").textContent="Scanning";const d=await api(`/api/network/scan${start?"?start=1":""}`);if(d.scanning){setTimeout(()=>scan(false),1200);return}$("networks").innerHTML=`<h2>Networks</h2>${d.networks.map(n=>`<button data-ssid="${esc(n.ssid)}">${esc(n.ssid)} (${n.rssi} dBm)</button>`).join("")||"No networks found"}`;document.querySelectorAll("[data-ssid]").forEach(b=>b.onclick=()=>{$("ssid").value=b.dataset.ssid;updateSummary()})}
document.querySelectorAll("input,select").forEach(el=>{el.oninput=updateSummary;el.onchange=updateSummary});document.querySelectorAll("[data-step]").forEach(b=>b.onclick=()=>{if(validate(step))showStep(Number(b.dataset.step))});$("back").onclick=()=>showStep(step-1);$("next").onclick=()=>{if(validate(step))showStep(step+1)};$("save").onclick=save;$("scan").onclick=()=>scan(true);load().then(()=>showStep(0)).catch(e=>$("status").textContent=e.message);
</script>
</body>
</html>
)HTML";

static const char* speedName(SpeedMode speed) {
    if (speed == SPEED_33) return "33 RPM";
    if (speed == SPEED_45) return "45 RPM";
    return "78 RPM";
}

static const char* motorStateName() {
    if (motor.isRelayTestMode()) return "RELAY TEST";
    switch (motor.getState()) {
        case STATE_STANDBY: return "STANDBY";
        case STATE_STOPPED: return "STOPPED";
        case STATE_STARTING: return "STARTING";
        case STATE_RUNNING: return motor.isSpeedRamping() ? "RAMPING" : "RUNNING";
        case STATE_STOPPING: return "BRAKING";
    }
    return "UNKNOWN";
}

static float clampFloatValue(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static int clampIntValue(int value, int minValue, int maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static void copyJsonString(char* target, size_t targetSize, JsonVariant value, bool allowEmpty) {
    if (value.isNull() || !target || targetSize == 0) return;
    const char* text = value.as<const char*>();
    if (!text) return;
    if (!allowEmpty && text[0] == 0) return;
    strncpy(target, text, targetSize - 1);
    target[targetSize - 1] = 0;
}

static void setBool(JsonObject obj, const char* key, bool& target) {
    JsonVariant value = obj[key];
    if (!value.isNull()) target = value.as<bool>();
}

static void setByte(JsonObject obj, const char* key, uint8_t& target, int minValue, int maxValue) {
    JsonVariant value = obj[key];
    if (!value.isNull()) target = (uint8_t)clampIntValue(value.as<int>(), minValue, maxValue);
}

static void setFloat(JsonObject obj, const char* key, float& target, float minValue, float maxValue) {
    JsonVariant value = obj[key];
    if (!value.isNull()) target = clampFloatValue(value.as<float>(), minValue, maxValue);
}

static bool parseIpString(const char* text, uint8_t out[4]) {
    if (!text || !out) return false;
    int a, b, c, d;
    char tail;
    if (sscanf(text, "%d.%d.%d.%d%c", &a, &b, &c, &d, &tail) != 4) return false;
    if (a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255 || d < 0 || d > 255) return false;
    out[0] = (uint8_t)a;
    out[1] = (uint8_t)b;
    out[2] = (uint8_t)c;
    out[3] = (uint8_t)d;
    return true;
}

static String ipBytesToString(const uint8_t bytes[4]) {
    char buffer[18];
    snprintf(buffer, sizeof(buffer), "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3]);
    return String(buffer);
}

static JsonObject addSchemaField(JsonArray fields,
                                 const char* key,
                                 const char* label,
                                 const char* type,
                                 const char* help = nullptr,
                                 const char* unit = nullptr,
                                 bool safety = false) {
    JsonObject field = fields.add<JsonObject>();
    field["k"] = key;
    field["l"] = label;
    field["t"] = type;
    if (help && help[0]) field["help"] = help;
    if (unit && unit[0]) field["unit"] = unit;
    if (safety) field["safety"] = true;
    return field;
}

static void addNumberField(JsonArray fields,
                           const char* key,
                           const char* label,
                           float minValue,
                           float maxValue,
                           float step,
                           const char* help = nullptr,
                           const char* unit = nullptr,
                           bool safety = false) {
    JsonObject field = addSchemaField(fields, key, label, "number", help, unit, safety);
    field["min"] = minValue;
    field["max"] = maxValue;
    field["step"] = step;
}

static void addSelectField(JsonArray fields,
                           const char* key,
                           const char* label,
                           const char* optionKey,
                           const char* help = nullptr,
                           bool safety = false) {
    JsonObject field = addSchemaField(fields, key, label, "select", help, nullptr, safety);
    field["o"] = optionKey;
}

static void addCheckboxField(JsonArray fields,
                             const char* key,
                             const char* label,
                             const char* help = nullptr,
                             bool safety = false) {
    addSchemaField(fields, key, label, "checkbox", help, nullptr, safety);
}

static void addTextField(JsonArray fields,
                         const char* key,
                         const char* label,
                         const char* help = nullptr,
                         uint16_t maxLength = 0,
                         const char* format = nullptr) {
    JsonObject field = addSchemaField(fields, key, label, "text", help);
    if (maxLength > 0) field["maxLength"] = maxLength;
    if (format && format[0]) field["format"] = format;
}

static void addPasswordField(JsonArray fields,
                             const char* key,
                             const char* label,
                             const char* help = nullptr,
                             uint16_t maxLength = 0,
                             uint8_t minLength = 0) {
    JsonObject field = addSchemaField(fields, key, label, "password", help);
    if (maxLength > 0) field["maxLength"] = maxLength;
    if (minLength > 0) field["minLength"] = minLength;
}

static void addOptionPair(JsonArray options, int value, const char* label) {
    JsonArray option = options.add<JsonArray>();
    option.add(value);
    option.add(label);
}

static JsonArray addFieldGroup(JsonArray groups, const char* title) {
    JsonArray group = groups.add<JsonArray>();
    group.add(title);
    return group.add<JsonArray>();
}

static bool presetSlotExists(int slot) {
    char path[24];
    snprintf(path, sizeof(path), "/preset_%d.bin", slot);
    return LittleFS.exists(path);
}

WebInterface::WebInterface()
    : _server(80),
      _started(false),
      _authExpiresMs(0) {
    _authToken[0] = 0;
}

void WebInterface::begin() {
    setupRoutes();
    _server.enableCORS(true);
    _server.begin();
    _started = true;
}

void WebInterface::update() {
    if (_started && networkManager.isServerAvailable()) {
        _server.handleClient();
    }
}

bool WebInterface::isStarted() const {
    return _started;
}

void WebInterface::setupRoutes() {
    _server.collectHeaders("X-TTControl-Token");
    _server.on("/", HTTP_GET, [this]() { handleRoot(); });
    _server.on("/api/auth", HTTP_GET, [this]() { handleAuthGet(); });
    _server.on("/api/auth", HTTP_POST, [this]() { handleAuthPost(); });
    _server.on("/api/preferences", HTTP_GET, [this]() { handlePreferencesGet(); });
    _server.on("/api/preferences", HTTP_POST, [this]() { handlePreferencesPost(); });
    _server.on("/api/schema", HTTP_GET, [this]() { handleSchemaGet(); });
    _server.on("/api/status", HTTP_GET, [this]() { handleStatus(); });
    _server.on("/api/events", HTTP_GET, [this]() { handleEventsGet(); });
    _server.on("/api/settings", HTTP_GET, [this]() { handleSettingsGet(); });
    _server.on("/api/settings", HTTP_POST, [this]() { handleSettingsPost(); });
    _server.on("/api/control", HTTP_POST, [this]() { handleControl(); });
    _server.on("/api/network", HTTP_GET, [this]() { handleNetworkGet(); });
    _server.on("/api/network", HTTP_POST, [this]() { handleNetworkPost(); });
    _server.on("/api/network/scan", HTTP_GET, [this]() { handleNetworkScan(); });
    _server.on("/api/diagnostics", HTTP_GET, [this]() { handleDiagnosticsGet(); });
    _server.on("/api/presets", HTTP_GET, [this]() { handlePresetsGet(); });
    _server.on("/api/preset", HTTP_POST, [this]() { handlePresetPost(); });
    _server.on("/api/errors", HTTP_GET, [this]() { handleErrorsGet(); });
    _server.on("/api/errors", HTTP_POST, [this]() { handleErrorsPost(); });
    _server.onNotFound([this]() { handleNotFound(); });
}

void WebInterface::sendJson(int code, JsonDocument& doc) {
    String out;
    serializeJson(doc, out);
    _server.sendHeader("Cache-Control", "no-store");
    _server.send(code, "application/json", out);
}

void WebInterface::sendError(int code, const char* message) {
    JsonDocument doc;
    doc["ok"] = false;
    doc["error"] = message;
    sendJson(code, doc);
}

bool WebInterface::parseBody(JsonDocument& doc) {
    String body = _server.arg("plain");
    if (body.length() == 0) {
        sendError(400, "Missing request body");
        return false;
    }

    DeserializationError error = deserializeJson(doc, body);
    if (error) {
        sendError(400, error.c_str());
        return false;
    }

    return true;
}

bool WebInterface::isOpenSetupRequest() {
    return networkManager.isSetupApOpen() &&
           _server.client().localIP().toString() == networkManager.apIpText();
}

bool WebInterface::rejectOpenSetupAccess() {
    if (!isOpenSetupRequest()) return false;
    sendError(403, "Open setup mode only allows Wi-Fi configuration");
    return true;
}

bool WebInterface::hasWriteAccess() {
    if (!networkManager.isWebAccessLocked()) return true;
    uint32_t now = millis();
    if (_authToken[0] == 0 || (int32_t)(_authExpiresMs - now) <= 0) {
        clearAuthToken();
        return false;
    }
    String token = _server.header("X-TTControl-Token");
    if (token.length() == 0) return false;
    if (strcmp(token.c_str(), _authToken) != 0) return false;
    _authExpiresMs = now + 30UL * 60UL * 1000UL;
    return true;
}

bool WebInterface::requireWriteAccess() {
    if (hasWriteAccess()) return false;
    sendError(401, "PIN unlock required");
    return true;
}

void WebInterface::clearAuthToken() {
    _authToken[0] = 0;
    _authExpiresMs = 0;
}

void WebInterface::issueAuthToken() {
    uint32_t a = random(0x7fffffff) ^ millis();
    uint32_t b = random(0x7fffffff) ^ micros();
    snprintf(_authToken, sizeof(_authToken), "%08lx%08lx", (unsigned long)a, (unsigned long)b);
    _authExpiresMs = millis() + 30UL * 60UL * 1000UL;
}

void WebInterface::handleRoot() {
    if (isOpenSetupRequest()) {
        handleSetupRoot();
        return;
    }

    _server.sendHeader("Cache-Control", "no-store");
    _server.send(200, "text/html", INDEX_HTML);
}

void WebInterface::handleSetupRoot() {
    _server.sendHeader("Cache-Control", "no-store");
    _server.send(200, "text/html", SETUP_HTML);
}

void WebInterface::handleAuthGet() {
    JsonDocument doc;
    const NetworkConfig& c = networkManager.getConfig();
    doc["required"] = networkManager.isWebAccessLocked();
    doc["readOnlyMode"] = c.readOnlyMode;
    doc["pinSet"] = c.webPin[0] != 0;
    doc["unlocked"] = hasWriteAccess();
    doc["expiresMs"] = _authExpiresMs;
    sendJson(200, doc);
}

void WebInterface::handleAuthPost() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;

    const char* action = doc["action"].as<const char*>();
    if (action && strcmp(action, "logout") == 0) {
        clearAuthToken();
        handleAuthGet();
        return;
    }

    if (!networkManager.isWebAccessLocked()) {
        clearAuthToken();
        handleAuthGet();
        return;
    }

    const char* pin = doc["pin"].as<const char*>();
    if (!networkManager.verifyWebPin(pin)) {
        clearAuthToken();
        sendError(401, "Incorrect PIN");
        return;
    }

    issueAuthToken();
    JsonDocument out;
    out["required"] = true;
    out["readOnlyMode"] = true;
    out["pinSet"] = true;
    out["unlocked"] = true;
    out["token"] = _authToken;
    out["expiresMs"] = _authExpiresMs;
    sendJson(200, out);
}

void WebInterface::handlePreferencesGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    const NetworkConfig& c = networkManager.getConfig();
    doc["homePage"] = c.webHomePage;
    sendJson(200, doc);
}

void WebInterface::handlePreferencesPost() {
    if (rejectOpenSetupAccess()) return;
    if (requireWriteAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;

    NetworkConfig& c = networkManager.getConfig();
    setByte(doc.as<JsonObject>(), "homePage", c.webHomePage, 0, WEB_HOME_PAGE_COUNT - 1);
    networkManager.save();
    handlePreferencesGet();
}

void WebInterface::handleSchemaGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    doc["settingsSchemaVersion"] = SETTINGS_SCHEMA_VERSION;
    doc["networkSchemaVersion"] = NETWORK_CONFIG_VERSION;

    JsonArray speedNames = doc["speedNames"].to<JsonArray>();
    speedNames.add("33 RPM");
    speedNames.add("45 RPM");
    speedNames.add("78 RPM");

    JsonObject options = doc["options"].to<JsonObject>();
    JsonArray phaseMode = options["phaseMode"].to<JsonArray>();
    addOptionPair(phaseMode, 1, "1 phase");
    addOptionPair(phaseMode, 2, "2 phase");
    addOptionPair(phaseMode, 3, "3 phase");
#if MAX_PHASE_MODE >= 4
    addOptionPair(phaseMode, 4, "4 phase");
#endif

    JsonArray filterType = options["filterType"].to<JsonArray>();
    addOptionPair(filterType, FILTER_NONE, "None");
    addOptionPair(filterType, FILTER_IIR, "IIR");
    addOptionPair(filterType, FILTER_FIR, "FIR");

    JsonArray firProfile = options["firProfile"].to<JsonArray>();
    addOptionPair(firProfile, FIR_GENTLE, "Gentle");
    addOptionPair(firProfile, FIR_MEDIUM, "Medium");
    addOptionPair(firProfile, FIR_AGGRESSIVE, "Aggressive");

    JsonArray softStartCurve = options["softStartCurve"].to<JsonArray>();
    addOptionPair(softStartCurve, 0, "Linear");
    addOptionPair(softStartCurve, 1, "Log");
    addOptionPair(softStartCurve, 2, "Exp");

    JsonArray rampType = options["rampType"].to<JsonArray>();
    addOptionPair(rampType, RAMP_LINEAR, "Linear");
    addOptionPair(rampType, RAMP_SCURVE, "S-curve");

    JsonArray brakeMode = options["brakeMode"].to<JsonArray>();
    addOptionPair(brakeMode, BRAKE_OFF, "Off");
    addOptionPair(brakeMode, BRAKE_PULSE, "Pulse");
    addOptionPair(brakeMode, BRAKE_RAMP, "Ramp");
    addOptionPair(brakeMode, BRAKE_SOFT_STOP, "Soft stop");

    JsonArray screensaverMode = options["screensaverMode"].to<JsonArray>();
    addOptionPair(screensaverMode, SAVER_BOUNCE, "Bounce");
    addOptionPair(screensaverMode, SAVER_MATRIX, "Matrix");
    addOptionPair(screensaverMode, SAVER_LISSAJOUS, "Lissajous");

    JsonArray displaySleepDelay = options["displaySleepDelay"].to<JsonArray>();
    addOptionPair(displaySleepDelay, 0, "Off");
    addOptionPair(displaySleepDelay, 1, "10 sec");
    addOptionPair(displaySleepDelay, 2, "20 sec");
    addOptionPair(displaySleepDelay, 3, "30 sec");
    addOptionPair(displaySleepDelay, 4, "1 min");
    addOptionPair(displaySleepDelay, 5, "5 min");
    addOptionPair(displaySleepDelay, 6, "10 min");

    JsonArray bootSpeed = options["bootSpeed"].to<JsonArray>();
    addOptionPair(bootSpeed, 0, "33 RPM");
    addOptionPair(bootSpeed, 1, "45 RPM");
    addOptionPair(bootSpeed, 2, "78 RPM");
    addOptionPair(bootSpeed, 3, "Last used");

    JsonArray netMode = options["netMode"].to<JsonArray>();
    addOptionPair(netMode, NETWORK_MODE_AP, "Setup AP");
    addOptionPair(netMode, NETWORK_MODE_STA, "Station");
    addOptionPair(netMode, NETWORK_MODE_STA_AP, "Station + setup AP");

    JsonArray homePage = options["homePage"].to<JsonArray>();
    addOptionPair(homePage, WEB_HOME_DASHBOARD, "Dashboard");
    addOptionPair(homePage, WEB_HOME_CONTROL, "Control");
    addOptionPair(homePage, WEB_HOME_SETTINGS, "Settings");
    addOptionPair(homePage, WEB_HOME_CALIBRATE, "Calibrate");
    addOptionPair(homePage, WEB_HOME_NETWORK, "Network");
    addOptionPair(homePage, WEB_HOME_PRESETS, "Presets");
    addOptionPair(homePage, WEB_HOME_BENCH, "Bench");
    addOptionPair(homePage, WEB_HOME_DIAGNOSTICS, "Diagnostics");
    addOptionPair(homePage, WEB_HOME_ERRORS, "Errors");

    JsonArray globalGroups = doc["globalGroups"].to<JsonArray>();
    JsonArray fields = addFieldGroup(globalGroups, "Phase");
    addSelectField(fields, "phaseMode", "Phase mode", "phaseMode", "Number of active phase outputs.", true);

    fields = addFieldGroup(globalGroups, "Motor");
    addNumberField(fields, "maxAmplitude", "Maximum amplitude", 0, 100, 1, "Upper output amplitude limit.", "percent", true);
    addNumberField(fields, "freqDependentAmplitude", "V/f blend", 0, 100, 1, "How strongly the V/f boost curve affects output amplitude.", "percent", true);
    addNumberField(fields, "vfLowFreq", "V/f low frequency", 0, 50, 1, "Low-frequency point for voltage boost.", "Hz", true);
    addNumberField(fields, "vfLowBoost", "V/f low boost", 0, 100, 1, "Output boost at the low-frequency V/f point.", "percent", true);
    addNumberField(fields, "vfMidFreq", "V/f mid frequency", 0, 100, 1, "Mid-frequency point for voltage boost.", "Hz", true);
    addNumberField(fields, "vfMidBoost", "V/f mid boost", 0, 100, 1, "Output boost at the mid-frequency V/f point.", "percent", true);
    addSelectField(fields, "rampType", "Ramp type", "rampType", "Acceleration curve used for speed changes.");
    addSelectField(fields, "softStartCurve", "Soft start curve", "softStartCurve", "Curve used during startup.");
    addCheckboxField(fields, "smoothSwitching", "Smooth speed switching", "Ramp between speed presets instead of stepping instantly.");
    addNumberField(fields, "switchRampDuration", "Switch ramp duration", 1, 5, 1, "Duration used when smooth speed switching is enabled.", "sec");
    addSelectField(fields, "brakeMode", "Brake mode", "brakeMode", "How the motor is stopped.", true);
    addNumberField(fields, "brakeDuration", "Brake duration", 0, 10, 0.1f, "Braking or ramp-down duration.", "sec", true);
    addNumberField(fields, "brakePulseGap", "Brake pulse gap", 0.1f, 2, 0.1f, "Gap between brake pulses.", "sec", true);
    addNumberField(fields, "brakeStartFreq", "Brake start frequency", 10, 200, 1, "Starting frequency for active braking.", "Hz", true);
    addNumberField(fields, "brakeStopFreq", "Brake stop frequency", 0, 50, 1, "Final frequency for active braking.", "Hz", true);
    addNumberField(fields, "softStopCutoff", "Soft stop cutoff", 0, 50, 1, "Frequency below which soft stop disables drive.", "Hz", true);
    addCheckboxField(fields, "autoStart", "Auto start", "Start automatically after waking from standby.", true);

    fields = addFieldGroup(globalGroups, "Power");
    addCheckboxField(fields, "relayActiveHigh", "Relay active high", "Enable when relay boards switch on with a high signal.", true);
    addCheckboxField(fields, "muteRelayLinkStandby", "Mute relays in standby", "Link mute relay state to standby.", true);
    addCheckboxField(fields, "muteRelayLinkStartStop", "Mute relays on stop", "Mute outputs when the motor is stopped.", true);
    addNumberField(fields, "powerOnRelayDelay", "Power-on relay delay", 0, 10, 1, "Delay before relays engage after power-on.", "sec", true);
    addNumberField(fields, "autoStandbyDelay", "Auto standby delay", 0, 60, 1, "Minutes of inactivity before standby. Zero disables it.", "min");
    addCheckboxField(fields, "autoBoot", "Auto boot to powered state", "Wake to powered state after boot.", true);

    fields = addFieldGroup(globalGroups, "Display");
    addNumberField(fields, "displayBrightness", "Display brightness", 0, 255, 1, "OLED brightness level.");
    addSelectField(fields, "displaySleepDelay", "Display sleep delay", "displaySleepDelay", "Delay before the display sleeps.");
    addCheckboxField(fields, "screensaverEnabled", "Screensaver enabled", "Show a screensaver after the sleep delay.");
    addSelectField(fields, "screensaverMode", "Screensaver mode", "screensaverMode", "Animation shown after the display sleep delay.");
    addNumberField(fields, "autoDimDelay", "Auto dim delay", 0, 60, 1, "Minutes before dimming. Zero disables it.", "min");
    addCheckboxField(fields, "showRuntime", "Show runtime", "Include runtime on the display.");
    addCheckboxField(fields, "errorDisplayEnabled", "Error display enabled", "Show errors on the OLED.");
    addNumberField(fields, "errorDisplayDuration", "Error display duration", 1, 60, 1, "How long OLED error messages stay visible.", "sec");

    fields = addFieldGroup(globalGroups, "System");
    addCheckboxField(fields, "reverseEncoder", "Reverse encoder", "Invert main encoder direction.");
    addNumberField(fields, "pitchStepSize", "Pitch step size", 0.01f, 1, 0.01f, "Pitch adjustment step size.", "Hz");
    addCheckboxField(fields, "pitchResetOnStop", "Pitch reset on stop", "Reset pitch after stopping.");
    addCheckboxField(fields, "enable78rpm", "Enable 78 RPM", "Expose and allow 78 RPM mode.");
#if AMP_MONITOR_ENABLE
    addNumberField(fields, "ampTempWarnC", "Amplifier warning temperature", AMP_TEMP_MIN_C, AMP_TEMP_MAX_C, 1, "Temperature that logs a non-critical amplifier thermal warning.", "C", true);
    addNumberField(fields, "ampTempShutdownC", "Amplifier shutdown temperature", AMP_TEMP_MIN_C, AMP_TEMP_MAX_C, 1, "Temperature that immediately stops the motor and latches amplifier thermal shutdown.", "C", true);
#endif
    addSelectField(fields, "bootSpeed", "Boot speed", "bootSpeed", "Speed selected at startup.");

    JsonArray speedFields = doc["speedFields"].to<JsonArray>();
    addNumberField(speedFields, "frequency", "Frequency", MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ, 0.1f, "Nominal drive frequency for this speed.", "Hz", true);
    addNumberField(speedFields, "minFrequency", "Minimum frequency", MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ, 0.1f, "Lower pitch/frequency limit for this speed.", "Hz", true);
    addNumberField(speedFields, "maxFrequency", "Maximum frequency", MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ, 0.1f, "Upper pitch/frequency limit for this speed.", "Hz", true);
    addNumberField(speedFields, "phase0", "Phase 1 offset", -360, 360, 0.1f, "Phase 1 offset from reference.", "deg", true);
    addNumberField(speedFields, "phase1", "Phase 2 offset", -360, 360, 0.1f, "Phase 2 offset from reference.", "deg", true);
    addNumberField(speedFields, "phase2", "Phase 3 offset", -360, 360, 0.1f, "Phase 3 offset from reference.", "deg", true);
    addNumberField(speedFields, "phase3", "Phase 4 offset", -360, 360, 0.1f, "Phase 4 offset from reference.", "deg", true);
    addNumberField(speedFields, "softStartDuration", "Soft start duration", 0, 10, 0.1f, "Startup ramp duration for this speed.", "sec", true);
    addNumberField(speedFields, "reducedAmplitude", "Reduced amplitude", 10, 100, 1, "Running amplitude after startup delay.", "percent", true);
    addNumberField(speedFields, "amplitudeDelay", "Amplitude delay", 0, 60, 1, "Delay before reducing amplitude.", "sec", true);
    addNumberField(speedFields, "startupKick", "Startup kick multiplier", 1, 4, 1, "Temporary startup torque multiplier.", "x", true);
    addNumberField(speedFields, "startupKickDuration", "Startup kick duration", 0, 15, 1, "How long startup kick stays active.", "sec", true);
    addNumberField(speedFields, "startupKickRampDuration", "Startup kick ramp", 0, 15, 0.1f, "Ramp-out duration for startup kick.", "sec", true);
    addSelectField(speedFields, "filterType", "Filter type", "filterType", "Digital smoothing filter for speed changes.");
    addNumberField(speedFields, "iirAlpha", "IIR alpha", 0.01f, 0.99f, 0.01f, "IIR smoothing coefficient.");
    addSelectField(speedFields, "firProfile", "FIR profile", "firProfile", "FIR smoothing profile.");

    JsonArray networkFields = doc["networkFields"].to<JsonArray>();
    addCheckboxField(networkFields, "enabled", "Wi-Fi enabled", "Turn Wi-Fi features on for this board.");
    addSelectField(networkFields, "mode", "Connection mode", "netMode", "Choose setup AP, Wi-Fi client, or both.");
    addTextField(networkFields, "hostname", "Hostname", "Local network host name.", NETWORK_HOSTNAME_MAX);
    addTextField(networkFields, "ssid", "Network SSID", "Wi-Fi network name.", NETWORK_SSID_MAX);
    addPasswordField(networkFields, "password", "Network password", "Leave blank to keep the saved password.", NETWORK_PASSWORD_MAX, 8);
    addCheckboxField(networkFields, "dhcp", "Use DHCP", "Automatically request an IP address.");
    addTextField(networkFields, "staticIp", "Static IP", "Used when DHCP is off.", 15, "ip");
    addTextField(networkFields, "gateway", "Gateway", "Used when DHCP is off.", 15, "ip");
    addTextField(networkFields, "subnet", "Subnet mask", "Used when DHCP is off.", 15, "ip");
    addTextField(networkFields, "dns", "DNS server", "Used when DHCP is off.", 15, "ip");
    addCheckboxField(networkFields, "apFallback", "Setup AP fallback", "Reopen setup AP when Wi-Fi fails.");
    addTextField(networkFields, "apSsid", "Setup AP SSID", "Name of the setup network.", NETWORK_SSID_MAX);
    addPasswordField(networkFields, "apPassword", "Setup AP password", "Leave blank for an open setup network.", NETWORK_PASSWORD_MAX, 8);
    addNumberField(networkFields, "apChannel", "Setup AP channel", 1, 13, 1, "Wi-Fi channel used by the setup AP.");
    addCheckboxField(networkFields, "readOnlyMode", "Read-only guest mode", "When enabled, dashboard/status stay visible but controls and settings require the web PIN.");
    addPasswordField(networkFields, "webPin", "Web PIN", "Leave blank to keep the saved PIN. Use 4 to 8 characters.", NETWORK_WEB_PIN_MAX, 4);
    addSelectField(networkFields, "webHomePage", "Web home page", "homePage", "Default page shown when the local browser interface opens.");

    sendJson(200, doc);
}

void WebInterface::populateStatus(JsonDocument& doc) {
    doc["firmware"] = FIRMWARE_VERSION;
    doc["buildDate"] = BUILD_DATE;
    doc["safeMode"] = safeModeActive;

    JsonObject motorJson = doc["motor"].to<JsonObject>();
    motorJson["state"] = motorStateName();
    motorJson["stateCode"] = (int)motor.getState();
    motorJson["running"] = motor.isRunning();
    motorJson["standby"] = motor.isStandby();
    motorJson["speed"] = (int)motor.getSpeed();
    motorJson["speedName"] = speedName(motor.getSpeed());
    motorJson["frequency"] = motor.getCurrentFrequency();
    motorJson["pitch"] = motor.getPitchPercent();
    motorJson["pitchRange"] = motor.getPitchRange();
    motorJson["motionProgress"] = motor.getMotionProgress();
    motorJson["speedRamping"] = motor.isSpeedRamping();
    motorJson["relayTest"] = motor.isRelayTestMode();
    motorJson["relayStage"] = motor.getRelayTestStage();
    motorJson["relayStageCount"] = motor.getRelayTestStageCount();

    JsonObject runtime = doc["runtime"].to<JsonObject>();
    runtime["session"] = settings.getSessionRuntime();
    runtime["total"] = settings.getTotalRuntime();

    JsonObject scope = doc["scope"].to<JsonObject>();
    scope["a"] = waveform.getSample(0);
    scope["b"] = waveform.getSample(1);

    JsonObject amp = doc["amp"].to<JsonObject>();
#if AMP_MONITOR_ENABLE
    amp["enabled"] = true;
    amp["temperatureC"] = ampMonitor.getTemperatureC();
    amp["thermalOk"] = ampMonitor.isThermalOk();
    amp["state"] = ampMonitor.isThermalOk() ? "OK" : "TRIPPED";
    amp["warnC"] = settings.get().ampTempWarnC;
    amp["shutdownC"] = settings.get().ampTempShutdownC;
#else
    amp["enabled"] = false;
#endif

    JsonObject net = doc["network"].to<JsonObject>();
    net["available"] = networkManager.isAvailable();
    net["enabled"] = networkManager.isEnabled();
    net["status"] = networkManager.statusText();
    net["ip"] = networkManager.ipText();
    net["ssid"] = networkManager.ssidText();
    net["rssi"] = networkManager.rssi();
    net["clients"] = networkManager.connectedClientCount();

    JsonObject auth = doc["auth"].to<JsonObject>();
    const NetworkConfig& cfg = networkManager.getConfig();
    auth["readOnlyMode"] = cfg.readOnlyMode;
    auth["required"] = networkManager.isWebAccessLocked();
    auth["unlocked"] = hasWriteAccess();
}

void WebInterface::handleStatus() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    populateStatus(doc);
    sendJson(200, doc);
}

void WebInterface::handleEventsGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    populateStatus(doc);
    String payload;
    serializeJson(doc, payload);
    String out = "retry: 1000\n";
    out += "event: status\n";
    out += "data: ";
    out += payload;
    out += "\n\n";

    _server.sendHeader("Cache-Control", "no-store");
    _server.send(200, "text/event-stream", out);
}

void WebInterface::handleSettingsGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    GlobalSettings& g = settings.get();
    JsonObject global = doc["global"].to<JsonObject>();

    global["schemaVersion"] = g.schemaVersion;
    global["phaseMode"] = g.phaseMode;
    global["maxAmplitude"] = g.maxAmplitude;
    global["softStartCurve"] = g.softStartCurve;
    global["smoothSwitching"] = g.smoothSwitching;
    global["switchRampDuration"] = g.switchRampDuration;
    global["brakeMode"] = g.brakeMode;
    global["brakeDuration"] = g.brakeDuration;
    global["brakePulseGap"] = g.brakePulseGap;
    global["brakeStartFreq"] = g.brakeStartFreq;
    global["brakeStopFreq"] = g.brakeStopFreq;
    global["softStopCutoff"] = g.softStopCutoff;
    global["relayActiveHigh"] = g.relayActiveHigh;
    global["muteRelayLinkStandby"] = g.muteRelayLinkStandby;
    global["muteRelayLinkStartStop"] = g.muteRelayLinkStartStop;
    global["powerOnRelayDelay"] = g.powerOnRelayDelay;
    global["displayBrightness"] = g.displayBrightness;
    global["displaySleepDelay"] = g.displaySleepDelay;
    global["screensaverEnabled"] = g.screensaverEnabled;
    global["autoDimDelay"] = g.autoDimDelay;
    global["showRuntime"] = g.showRuntime;
    global["errorDisplayEnabled"] = g.errorDisplayEnabled;
    global["errorDisplayDuration"] = g.errorDisplayDuration;
    global["autoStandbyDelay"] = g.autoStandbyDelay;
    global["autoStart"] = g.autoStart;
    global["autoBoot"] = g.autoBoot;
    global["pitchResetOnStop"] = g.pitchResetOnStop;
    global["reverseEncoder"] = g.reverseEncoder;
    global["pitchStepSize"] = g.pitchStepSize;
    global["rampType"] = g.rampType;
    global["screensaverMode"] = g.screensaverMode;
    global["enable78rpm"] = g.enable78rpm;
    global["freqDependentAmplitude"] = g.freqDependentAmplitude;
    global["vfLowFreq"] = g.vfLowFreq;
    global["vfLowBoost"] = g.vfLowBoost;
    global["vfMidFreq"] = g.vfMidFreq;
    global["vfMidBoost"] = g.vfMidBoost;
    global["bootSpeed"] = g.bootSpeed;
#if AMP_MONITOR_ENABLE
    global["ampTempWarnC"] = g.ampTempWarnC;
    global["ampTempShutdownC"] = g.ampTempShutdownC;
#endif
    global["totalRuntime"] = g.totalRuntime;

    JsonArray speeds = doc["speeds"].to<JsonArray>();
    for (int i = 0; i < 3; i++) {
        SpeedSettings& s = g.speeds[i];
        JsonObject speed = speeds.add<JsonObject>();
        speed["frequency"] = s.frequency;
        speed["minFrequency"] = s.minFrequency;
        speed["maxFrequency"] = s.maxFrequency;
        JsonArray phase = speed["phaseOffset"].to<JsonArray>();
        for (int p = 0; p < 4; p++) phase.add(s.phaseOffset[p]);
        speed["softStartDuration"] = s.softStartDuration;
        speed["reducedAmplitude"] = s.reducedAmplitude;
        speed["amplitudeDelay"] = s.amplitudeDelay;
        speed["startupKick"] = s.startupKick;
        speed["startupKickDuration"] = s.startupKickDuration;
        speed["startupKickRampDuration"] = s.startupKickRampDuration;
        speed["filterType"] = s.filterType;
        speed["iirAlpha"] = s.iirAlpha;
        speed["firProfile"] = s.firProfile;
    }

    sendJson(200, doc);
}

void WebInterface::handleSettingsPost() {
    if (rejectOpenSetupAccess()) return;
    if (requireWriteAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;

    GlobalSettings& g = settings.get();
    JsonObject global = doc["global"].as<JsonObject>();
    if (!global.isNull()) {
        setByte(global, "phaseMode", g.phaseMode, 1, MAX_PHASE_MODE);
        setByte(global, "maxAmplitude", g.maxAmplitude, 0, 100);
        setByte(global, "softStartCurve", g.softStartCurve, 0, 2);
        setBool(global, "smoothSwitching", g.smoothSwitching);
        setByte(global, "switchRampDuration", g.switchRampDuration, 1, 5);
        setByte(global, "brakeMode", g.brakeMode, 0, BRAKE_SOFT_STOP);
        setFloat(global, "brakeDuration", g.brakeDuration, 0.0f, 10.0f);
        setFloat(global, "brakePulseGap", g.brakePulseGap, 0.1f, 2.0f);
        setFloat(global, "brakeStartFreq", g.brakeStartFreq, 10.0f, 200.0f);
        setFloat(global, "brakeStopFreq", g.brakeStopFreq, 0.0f, 50.0f);
        setFloat(global, "softStopCutoff", g.softStopCutoff, 0.0f, 50.0f);
        setBool(global, "relayActiveHigh", g.relayActiveHigh);
        setBool(global, "muteRelayLinkStandby", g.muteRelayLinkStandby);
        setBool(global, "muteRelayLinkStartStop", g.muteRelayLinkStartStop);
        setByte(global, "powerOnRelayDelay", g.powerOnRelayDelay, 0, 10);
        setByte(global, "displayBrightness", g.displayBrightness, 0, 255);
        setByte(global, "displaySleepDelay", g.displaySleepDelay, 0, 6);
        setBool(global, "screensaverEnabled", g.screensaverEnabled);
        setByte(global, "autoDimDelay", g.autoDimDelay, 0, 60);
        setBool(global, "showRuntime", g.showRuntime);
        setBool(global, "errorDisplayEnabled", g.errorDisplayEnabled);
        setByte(global, "errorDisplayDuration", g.errorDisplayDuration, 1, 60);
        setByte(global, "autoStandbyDelay", g.autoStandbyDelay, 0, 60);
        setBool(global, "autoStart", g.autoStart);
        setBool(global, "autoBoot", g.autoBoot);
        setBool(global, "pitchResetOnStop", g.pitchResetOnStop);
        setBool(global, "reverseEncoder", g.reverseEncoder);
        setFloat(global, "pitchStepSize", g.pitchStepSize, 0.01f, 1.0f);
        setByte(global, "rampType", g.rampType, RAMP_LINEAR, RAMP_SCURVE);
        setByte(global, "screensaverMode", g.screensaverMode, SAVER_BOUNCE, SAVER_LISSAJOUS);
        setBool(global, "enable78rpm", g.enable78rpm);
        setByte(global, "freqDependentAmplitude", g.freqDependentAmplitude, 0, 100);
        setFloat(global, "vfLowFreq", g.vfLowFreq, 0.0f, 50.0f);
        setByte(global, "vfLowBoost", g.vfLowBoost, 0, 100);
        setFloat(global, "vfMidFreq", g.vfMidFreq, 0.0f, 100.0f);
        setByte(global, "vfMidBoost", g.vfMidBoost, 0, 100);
        setByte(global, "bootSpeed", g.bootSpeed, 0, 3);
#if AMP_MONITOR_ENABLE
        setFloat(global, "ampTempWarnC", g.ampTempWarnC, AMP_TEMP_MIN_C, AMP_TEMP_MAX_C);
        setFloat(global, "ampTempShutdownC", g.ampTempShutdownC, AMP_TEMP_MIN_C, AMP_TEMP_MAX_C);
#endif
    }

    JsonArray speeds = doc["speeds"].as<JsonArray>();
    if (!speeds.isNull()) {
        for (size_t i = 0; i < 3 && i < speeds.size(); i++) {
            JsonObject speed = speeds[i].as<JsonObject>();
            if (speed.isNull()) continue;
            SpeedSettings& s = g.speeds[i];
            setFloat(speed, "frequency", s.frequency, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ);
            setFloat(speed, "minFrequency", s.minFrequency, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ);
            setFloat(speed, "maxFrequency", s.maxFrequency, MIN_OUTPUT_FREQUENCY_HZ, MAX_OUTPUT_FREQUENCY_HZ);
            JsonArray phase = speed["phaseOffset"].as<JsonArray>();
            if (!phase.isNull()) {
                for (size_t p = 0; p < 4 && p < phase.size(); p++) {
                    s.phaseOffset[p] = clampFloatValue(phase[p].as<float>(), -360.0f, 360.0f);
                }
            }
            setFloat(speed, "softStartDuration", s.softStartDuration, 0.0f, 10.0f);
            setByte(speed, "reducedAmplitude", s.reducedAmplitude, 10, 100);
            setByte(speed, "amplitudeDelay", s.amplitudeDelay, 0, 60);
            setByte(speed, "startupKick", s.startupKick, 1, 4);
            setByte(speed, "startupKickDuration", s.startupKickDuration, 0, 15);
            setFloat(speed, "startupKickRampDuration", s.startupKickRampDuration, 0.0f, 15.0f);
            setByte(speed, "filterType", s.filterType, FILTER_NONE, FILTER_FIR);
            setFloat(speed, "iirAlpha", s.iirAlpha, 0.01f, 0.99f);
            setByte(speed, "firProfile", s.firProfile, FIR_GENTLE, FIR_AGGRESSIVE);
        }
    }

    g.schemaVersion = SETTINGS_SCHEMA_VERSION;
    settings.normalize();
    motor.applySettings();
    if (doc["save"].as<bool>()) {
        settings.save();
    }

    handleSettingsGet();
}

void WebInterface::handleControl() {
    if (rejectOpenSetupAccess()) return;
    if (requireWriteAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;
    const char* action = doc["action"].as<const char*>();
    if (!action) {
        sendError(400, "Missing action");
        return;
    }

    if (strcmp(action, "start") == 0) {
        if (motor.isRelayTestMode()) {
            sendError(409, "Exit relay test before starting");
            return;
        }
        if (motor.isStandby()) motor.toggleStandby();
        motor.start();
    } else if (strcmp(action, "emergencyStop") == 0) {
        motor.endRelayTest();
        motor.emergencyStop();
    } else if (strcmp(action, "stop") == 0) {
        if (motor.isRelayTestMode()) motor.endRelayTest();
        motor.stop();
    } else if (strcmp(action, "toggleStartStop") == 0) {
        if (motor.isStandby()) motor.toggleStandby();
        else motor.toggleStartStop();
    } else if (strcmp(action, "toggleStandby") == 0) {
        motor.toggleStandby();
    } else if (strcmp(action, "cycleSpeed") == 0) {
        motor.cycleSpeed();
    } else if (strcmp(action, "setSpeed") == 0) {
        int speed = clampIntValue(doc["speed"].as<int>(), 0, 2);
        if (speed == SPEED_78 && !settings.get().enable78rpm) {
            sendError(409, "78 RPM is disabled");
            return;
        }
        motor.setSpeed((SpeedMode)speed);
    } else if (strcmp(action, "resetPitch") == 0) {
        motor.resetPitch();
    } else if (strcmp(action, "setPitch") == 0) {
        motor.setPitch(doc["pitch"].as<float>());
    } else if (strcmp(action, "relayTest") == 0) {
        uint8_t stage = (uint8_t)clampIntValue(doc["stage"].as<int>(), 0, motor.getRelayTestStageCount() - 1);
        if (!motor.isRelayTestMode() && !motor.beginRelayTest()) {
            sendError(409, "Stop motor before relay test");
            return;
        }
        motor.setRelayTestStage(stage);
    } else if (strcmp(action, "relayOff") == 0) {
        motor.endRelayTest();
    } else if (strcmp(action, "resetRuntime") == 0) {
        settings.resetTotalRuntime();
        settings.resetSessionRuntime();
    } else if (strcmp(action, "factoryReset") == 0) {
        settings.factoryReset();
        settings.load();
        motor.endRelayTest();
        motor.applySettings();
        networkManager.resetDefaults();
        networkManager.restart();
    } else if (strcmp(action, "reboot") == 0) {
        hal.watchdogReboot();
    } else {
        sendError(400, "Unknown action");
        return;
    }

    JsonDocument out;
    out["ok"] = true;
    sendJson(200, out);
}

void WebInterface::handleNetworkGet() {
    JsonDocument doc;
    NetworkConfig& c = networkManager.getConfig();
    doc["available"] = networkManager.isAvailable();
    doc["enabled"] = networkManager.isEnabled();
    doc["connected"] = networkManager.isConnected();
    doc["apActive"] = networkManager.isApActive();
    doc["status"] = networkManager.statusText();
    doc["modeText"] = networkManager.modeText();
    doc["ip"] = networkManager.ipText();
    doc["stationIp"] = networkManager.stationIpText();
    doc["apIp"] = networkManager.apIpText();
    doc["ssid"] = networkManager.ssidText();
    doc["mac"] = networkManager.macText();
    doc["rssi"] = networkManager.rssi();
    doc["clients"] = networkManager.connectedClientCount();

    JsonObject config = doc["config"].to<JsonObject>();
    config["enabled"] = c.enabled;
    config["mode"] = c.mode;
    config["hostname"] = c.hostname;
    config["ssid"] = c.ssid;
    config["password"] = "";
    config["passwordSet"] = c.password[0] != 0;
    config["dhcp"] = c.dhcp;
    config["staticIp"] = ipBytesToString(c.staticIp);
    config["gateway"] = ipBytesToString(c.gateway);
    config["subnet"] = ipBytesToString(c.subnet);
    config["dns"] = ipBytesToString(c.dns);
    config["apFallback"] = c.apFallback;
    config["apSsid"] = c.apSsid;
    config["apPassword"] = "";
    config["apPasswordSet"] = c.apPassword[0] != 0;
    config["apChannel"] = c.apChannel;
    config["readOnlyMode"] = c.readOnlyMode;
    config["webPin"] = "";
    config["webPinSet"] = c.webPin[0] != 0;
    config["webHomePage"] = c.webHomePage;

    sendJson(200, doc);
}

void WebInterface::handleNetworkPost() {
    bool openSetup = isOpenSetupRequest();
    if (!openSetup && requireWriteAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;

    NetworkConfig& c = networkManager.getConfig();
    setBool(doc.as<JsonObject>(), "enabled", c.enabled);
    setByte(doc.as<JsonObject>(), "mode", c.mode, NETWORK_MODE_AP, NETWORK_MODE_STA_AP);
    setBool(doc.as<JsonObject>(), "dhcp", c.dhcp);
    setBool(doc.as<JsonObject>(), "apFallback", c.apFallback);
    setByte(doc.as<JsonObject>(), "apChannel", c.apChannel, 1, 13);
    copyJsonString(c.hostname, sizeof(c.hostname), doc["hostname"], false);
    copyJsonString(c.ssid, sizeof(c.ssid), doc["ssid"], true);
    copyJsonString(c.password, sizeof(c.password), doc["password"], true);
    copyJsonString(c.apSsid, sizeof(c.apSsid), doc["apSsid"], false);
    copyJsonString(c.apPassword, sizeof(c.apPassword), doc["apPassword"], true);
    if (!openSetup) {
        setBool(doc.as<JsonObject>(), "readOnlyMode", c.readOnlyMode);
        setByte(doc.as<JsonObject>(), "webHomePage", c.webHomePage, 0, WEB_HOME_PAGE_COUNT - 1);
        const char* pin = doc["webPin"].as<const char*>();
        if (pin && pin[0]) {
            size_t pinLen = strlen(pin);
            if (pinLen < 4 || pinLen > NETWORK_WEB_PIN_MAX) {
                sendError(400, "Web PIN must be 4 to 8 characters");
                return;
            }
            copyJsonString(c.webPin, sizeof(c.webPin), doc["webPin"], false);
            clearAuthToken();
        }
        if (c.readOnlyMode && c.webPin[0] == 0) {
            strncpy(c.webPin, NETWORK_DEFAULT_WEB_PIN, sizeof(c.webPin) - 1);
            c.webPin[sizeof(c.webPin) - 1] = 0;
        }
        if (!c.readOnlyMode) clearAuthToken();
    }

    if (!doc["staticIp"].isNull()) parseIpString(doc["staticIp"].as<const char*>(), c.staticIp);
    if (!doc["gateway"].isNull()) parseIpString(doc["gateway"].as<const char*>(), c.gateway);
    if (!doc["subnet"].isNull()) parseIpString(doc["subnet"].as<const char*>(), c.subnet);
    if (!doc["dns"].isNull()) parseIpString(doc["dns"].as<const char*>(), c.dns);

    networkManager.save();
    networkManager.restart();
    handleNetworkGet();
}

void WebInterface::handleNetworkScan() {
    static bool scanStarted = false;
    JsonDocument doc;

    if (_server.hasArg("start") || !scanStarted) {
        WiFi.scanDelete();
        WiFi.scanNetworks(true);
        scanStarted = true;
        doc["scanning"] = true;
        sendJson(200, doc);
        return;
    }

    int count = WiFi.scanComplete();
    if (count < 0) {
        doc["scanning"] = true;
        sendJson(200, doc);
        return;
    }

    doc["scanning"] = false;
    JsonArray networks = doc["networks"].to<JsonArray>();
    for (int i = 0; i < count; i++) {
        JsonObject network = networks.add<JsonObject>();
        const char* ssid = WiFi.SSID(i);
        network["ssid"] = ssid ? ssid : "";
        network["rssi"] = WiFi.RSSI(i);
        network["channel"] = WiFi.channel(i);
        network["encryption"] = WiFi.encryptionType(i);
    }
    WiFi.scanDelete();
    scanStarted = false;
    sendJson(200, doc);
}

void WebInterface::handleDiagnosticsGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    doc["firmware"] = FIRMWARE_VERSION;
    doc["buildDate"] = BUILD_DATE;
    doc["safeMode"] = safeModeActive;

    JsonObject flags = doc["flags"].to<JsonObject>();
    flags["NETWORK_ENABLE"] = NETWORK_ENABLE;
    flags["AMP_MONITOR_ENABLE"] = AMP_MONITOR_ENABLE;
    flags["ENABLE_STANDBY"] = ENABLE_STANDBY;
    flags["ENABLE_MUTE_RELAYS"] = ENABLE_MUTE_RELAYS;
    flags["ENABLE_DPDT_RELAYS"] = ENABLE_DPDT_RELAYS;
    flags["ENABLE_4_CHANNEL_SUPPORT"] = ENABLE_4_CHANNEL_SUPPORT;
    flags["PITCH_CONTROL_ENABLE"] = PITCH_CONTROL_ENABLE;
    flags["SERIAL_MONITOR_ENABLE"] = SERIAL_MONITOR_ENABLE;
    flags["SETTINGS_SCHEMA_VERSION"] = SETTINGS_SCHEMA_VERSION;
    flags["NETWORK_CONFIG_VERSION"] = NETWORK_CONFIG_VERSION;

    JsonObject pins = doc["pins"].to<JsonObject>();
    pins["PWM A"] = PIN_PWM_PHASE_A;
    pins["PWM B"] = PIN_PWM_PHASE_B;
    pins["PWM C"] = PIN_PWM_PHASE_C;
    pins["PWM D"] = PIN_PWM_PHASE_D;
    pins["Relay standby"] = PIN_RELAY_STANDBY;
    pins["Mute A"] = PIN_MUTE_PHASE_A;
    pins["Mute B"] = PIN_MUTE_PHASE_B;
    pins["Mute C"] = PIN_MUTE_PHASE_C;
    pins["Mute D"] = PIN_MUTE_PHASE_D;
    pins["Amp temp"] = PIN_AMP_TEMP;
    pins["Amp thermal OK"] = PIN_AMP_THERM_OK;

    JsonObject network = doc["network"].to<JsonObject>();
    network["status"] = networkManager.statusText();
    network["mode"] = networkManager.modeText();
    network["ip"] = networkManager.ipText();
    network["stationIp"] = networkManager.stationIpText();
    network["apIp"] = networkManager.apIpText();
    network["ssid"] = networkManager.ssidText();
    network["mac"] = networkManager.macText();
    network["rssi"] = networkManager.rssi();
    network["clients"] = networkManager.connectedClientCount();
    network["readOnlyMode"] = networkManager.getConfig().readOnlyMode;

    JsonObject files = doc["files"].to<JsonObject>();
    files["settings"] = LittleFS.exists("/settings.bin");
    files["network"] = LittleFS.exists("/network.bin");
    files["errors"] = LittleFS.exists("/error.log");
    JsonArray presets = files["presets"].to<JsonArray>();
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        JsonObject preset = presets.add<JsonObject>();
        char path[24];
        snprintf(path, sizeof(path), "/preset_%d.bin", i);
        preset["slot"] = i;
        preset["stored"] = LittleFS.exists(path);
    }

    JsonObject amp = doc["amp"].to<JsonObject>();
#if AMP_MONITOR_ENABLE
    amp["enabled"] = true;
    amp["temperatureC"] = ampMonitor.getTemperatureC();
    amp["thermalOk"] = ampMonitor.isThermalOk();
    amp["shutdown"] = ampMonitor.isShutdown();
    amp["warnC"] = settings.get().ampTempWarnC;
    amp["shutdownC"] = settings.get().ampTempShutdownC;
#else
    amp["enabled"] = false;
#endif

    sendJson(200, doc);
}

void WebInterface::handlePresetsGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    JsonArray presets = doc["presets"].to<JsonArray>();
    for (int i = 0; i < MAX_PRESET_SLOTS; i++) {
        JsonObject preset = presets.add<JsonObject>();
        char path[24];
        snprintf(path, sizeof(path), "/preset_%d.bin", i);
        preset["slot"] = i;
        preset["name"] = settings.getPresetName(i);
        preset["stored"] = LittleFS.exists(path);
    }
    sendJson(200, doc);
}

void WebInterface::handlePresetPost() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    if (!parseBody(doc)) return;
    int slot = clampIntValue(doc["slot"].as<int>(), 0, MAX_PRESET_SLOTS - 1);
    const char* action = doc["action"].as<const char*>();
    if (!action) {
        sendError(400, "Missing action");
        return;
    }

    bool readOnlyAction = strcmp(action, "export") == 0;
    if (!readOnlyAction && requireWriteAccess()) return;

    JsonDocument out;
    out["ok"] = true;
    if (strcmp(action, "load") == 0) {
        if (!settings.loadPreset(slot)) {
            sendError(404, "Preset slot is empty");
            return;
        }
        motor.applySettings();
        settings.save();
    } else if (strcmp(action, "save") == 0) {
        settings.savePreset(slot);
    } else if (strcmp(action, "rename") == 0) {
        const char* name = doc["name"].as<const char*>();
        if (!name) name = "";
        settings.renamePreset(slot, name);
    } else if (strcmp(action, "clear") == 0) {
        settings.resetPreset(slot);
    } else if (strcmp(action, "export") == 0) {
        if (!presetSlotExists(slot)) {
            sendError(404, "Preset slot is empty");
            return;
        }
        String exported;
        if (!settings.exportPresetToJSON(slot, exported)) {
            sendError(500, "Preset export failed");
            return;
        }
        out["json"] = exported;
    } else if (strcmp(action, "import") == 0) {
        const char* json = doc["json"].as<const char*>();
        if (!json || !settings.importPresetFromJSON(slot, String(json))) {
            sendError(400, "Preset import failed");
            return;
        }
    } else {
        sendError(400, "Unknown preset action");
        return;
    }
    sendJson(200, out);
}

void WebInterface::handleErrorsGet() {
    if (rejectOpenSetupAccess()) return;

    JsonDocument doc;
    std::vector<String> lines;
    errorHandler.getLogLines(lines);
    JsonArray out = doc["lines"].to<JsonArray>();
    for (const auto& line : lines) out.add(line);
    doc["critical"] = errorHandler.hasCriticalError();
    sendJson(200, doc);
}

void WebInterface::handleErrorsPost() {
    if (rejectOpenSetupAccess()) return;
    if (requireWriteAccess()) return;

    errorHandler.clearLogs();
    JsonDocument doc;
    doc["ok"] = true;
    sendJson(200, doc);
}

void WebInterface::handleNotFound() {
    if (_server.uri().startsWith("/api/")) {
        sendError(404, "Not found");
        return;
    }

    handleRoot();
}

#else

WebInterface::WebInterface()
    : _started(false) {
}

void WebInterface::begin() {
    _started = false;
}

void WebInterface::update() {
}

bool WebInterface::isStarted() const {
    return false;
}

#endif

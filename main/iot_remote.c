#include "iot_remote.h"

#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "homekit_accessory.h"

static const char *TAG = "iot_remote";

#define COMMAND_MAX_LEN 96
#define RESPONSE_MAX_LEN 640
#define MQTT_URI_MAX_LEN 128
#define MQTT_TOPIC_MAX_LEN 96
#define WIFI_SSID_MAX_LEN 32
#define WIFI_PASSWORD_MAX_LEN 64
#define WIFI_FORM_MAX_LEN 384
#define WIFI_CREDENTIAL_SLOTS 3
#define DEVICE_HOSTNAME_MAX_LEN 32
#define DEVICE_HOSTNAME_DEFAULT "esp-arc452a21"
#define HTTP_URI_HANDLER_SLOTS 14
#define DNS_PORT 53
#define DNS_PACKET_MAX_LEN 512

typedef struct {
    char ssid[WIFI_SSID_MAX_LEN + 1];
    char password[WIFI_PASSWORD_MAX_LEN + 1];
} wifi_credential_t;

static iot_remote_config_t s_config;
static httpd_handle_t s_httpd;
static esp_mqtt_client_handle_t s_mqtt;
static esp_netif_t *s_sta_netif;
static esp_netif_t *s_ap_netif;
static char s_mqtt_command_topic[MQTT_TOPIC_MAX_LEN];
static char s_mqtt_status_topic[MQTT_TOPIC_MAX_LEN];
static char s_mqtt_availability_topic[MQTT_TOPIC_MAX_LEN];
static char s_mqtt_uri[MQTT_URI_MAX_LEN];
static bool s_mqtt_enabled;
static char s_device_hostname[DEVICE_HOSTNAME_MAX_LEN + 1] = DEVICE_HOSTNAME_DEFAULT;
static wifi_credential_t s_wifi_credentials[WIFI_CREDENTIAL_SLOTS];
static size_t s_wifi_credential_count;
static size_t s_current_wifi_credential;
static bool s_ap_started;
static bool s_sta_connected;
static int s_sta_retries;
static TaskHandle_t s_dns_task;

static esp_err_t start_provisioning_ap(void);

static const char s_control_page[] =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>ESP-ARC452A21</title><style>"
    "body{font:15px system-ui,-apple-system,Segoe UI,sans-serif;margin:0;background:#f7f8fa;color:#171b22}"
    "main{max-width:720px;margin:auto;padding:18px}header{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;margin-bottom:12px}"
    "h1{font-size:22px;margin:0}.brand{display:grid;gap:2px}.projectLink{font-size:12px;color:#46515f}a{color:#1769e0;text-decoration:none}section{background:white;border:1px solid #d9dee5;border-radius:8px;padding:14px}"
    ".summary{display:flex;align-items:end;justify-content:space-between;gap:12px;margin-bottom:14px}.temp{font-size:42px;font-weight:700}.meta{color:#46515f;margin-top:3px}"
    ".powerBox{display:grid;gap:8px;justify-items:end}.powerSeg{justify-content:flex-end}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px}.field{display:grid;gap:6px;color:#46515f;font-size:13px}"
    "select,input{font:inherit;padding:10px;border:1px solid #c6ccd4;border-radius:6px;background:white}button{font:inherit;padding:10px 12px;border:0;border-radius:6px;background:#1769e0;color:white}"
    "button.secondary{background:#3d4856}.controls{display:grid;gap:12px;margin-top:14px}.controlRow{display:grid;grid-template-columns:110px 1fr;gap:10px;align-items:center}"
    ".controlLabel{color:#46515f;font-weight:600}.seg{display:flex;flex-wrap:wrap;gap:6px}.seg button{background:#eef2f7;color:#1c2633}.seg button.active{background:#1769e0;color:white}"
    ".stepper{display:grid;grid-template-columns:52px 1fr 52px;gap:8px}.stepper button{font-size:22px;padding:9px 0}.stepper output{display:grid;place-items:center;border:1px solid #c6ccd4;border-radius:6px;font-size:20px;font-weight:700;min-height:44px}"
    "#status{min-height:22px;color:#263241}#tempField.hidden{display:none}@media(max-width:520px){#tempField{grid-column:1/-1}.stepper{grid-template-columns:58px 1fr 58px}.controlRow{grid-template-columns:1fr;gap:6px}.controlRow .seg button{flex:1 1 auto}}"
    "</style></head><body><main><header><div class=brand><h1 id=hostTitle>esp-arc452a21</h1><a class=projectLink href='https://github.com/sohampatwardhan/ESP-ARC452A21'>ESP-ARC452A21</a></div><a href=/settings>Settings</a></header>"
    "<section><div class=summary><div><div class=temp id=summaryTemp>--</div><div class=meta id=summaryMeta>Last sent settings</div></div>"
    "<div class=powerBox><div id=powerState class=meta>--</div><div class='seg powerSeg' id=powerSeg><button data-v=on>On</button><button data-v=off>Off</button></div></div></div>"
    "<div class=grid><label class=field>Mode<select id=mode onchange=\"changed('mode')\"><option>cool</option><option>heat</option><option>auto</option><option>dry</option><option>fan</option></select></label>"
    "<label class=field>Fan<select id=fan onchange=\"changed('fan')\"><option>auto</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option><option>night</option></select></label>"
    "<div class=field id=tempField>Temperature<div class=stepper><button onclick=tempStep(-1)>-</button><output id=temp>72</output><button onclick=tempStep(1)>+</button></div></div></div>"
    "<div class=controls>"
    "<div class=controlRow><div class=controlLabel>Swing</div><div class=seg><button id=hswing data-toggle=hswing>Horizontal</button><button id=vswing data-toggle=vswing>Vertical</button></div></div>"
    "<div class=controlRow><div class=controlLabel>Quiet Mode</div><div class=seg id=quiet><button data-v=on>On</button><button data-v=off>Off</button></div></div>"
    "<div class=controlRow><div class=controlLabel>Sensor</div><div class=seg id=sensor><button data-sensor=comfort>Comfort</button><button data-sensor=eye>Intelligent Eye</button></div></div></div>"
    "<p id=status></p></section>"
    "</main><script>"
    "const $=id=>document.getElementById(id);let unit='f',ready=0,timer=0,lastUpdate=0,state={vswing:'off',hswing:'off',quiet:'off',sensor:'off'};const tempModes={auto:1,cool:1,heat:1},deg=String.fromCharCode(176);"
    "function esc(s){return String(s).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]))};"
    "fetch('/wifi').then(r=>r.json()).then(j=>{if(j.hostname){$('hostTitle').textContent=j.hostname;document.title=j.hostname}}).catch(()=>{});"
    "async function cmd(c){$('status').textContent='Sending...';let r=await fetch('/command',{method:'POST',body:c});let j=await r.json().catch(()=>({ok:false,error:'bad response'}));show(j)};"
    "function setSeg(id,v){state[id]=v;document.querySelectorAll('#'+id+' button').forEach(b=>b.classList.toggle('active',b.dataset.v==v))};"
    "function setToggle(id,on){state[id]=on?'on':'off';$(id).classList.toggle('active',on)};"
    "function setSensor(v){state.sensor=v;$('sensor').querySelector('[data-sensor=comfort]').classList.toggle('active',v=='comfort'||v=='both');$('sensor').querySelector('[data-sensor=eye]').classList.toggle('active',v=='eye'||v=='both')};"
    "function sensorValue(){let c=$('sensor').querySelector('[data-sensor=comfort]').classList.contains('active'),e=$('sensor').querySelector('[data-sensor=eye]').classList.contains('active');return c&&e?'both':c?'comfort':e?'eye':'off'};"
    "function setPower(v){$('powerState').textContent=v;document.querySelectorAll('#powerSeg button').forEach(b=>b.classList.toggle('active',b.dataset.v==v))};"
    "function updateAge(){if(!lastUpdate)return;let s=Math.floor((Date.now()-lastUpdate)/1000),m=Math.floor(s/60),h=Math.floor(m/60);$('status').textContent='Last updated '+(s<60?'just now':m<60?m+' min ago':h+' hr ago')};"
    "function queue(c){if(!ready)return;clearTimeout(timer);timer=setTimeout(()=>cmd(c),350)};"
    "['vswing','hswing'].forEach(id=>$(id).onclick=()=>{let on=state[id]!='on';setToggle(id,on);queue(id+' '+(on?'on':'off'))});"
    "$('quiet').onclick=e=>{if(e.target.dataset.v){setSeg('quiet',e.target.dataset.v);queue('quiet '+e.target.dataset.v)}};"
    "$('sensor').onclick=e=>{if(e.target.dataset.sensor){e.target.classList.toggle('active');let v=sensorValue();state.sensor=v;queue('sensor '+v)}};"
    "$('powerSeg').onclick=e=>{if(e.target.dataset.v)power(e.target.dataset.v)};"
    "function modeChanged(){$('tempField').classList.toggle('hidden',!tempModes[$('mode').value])};"
    "function tempBounds(){return unit=='c'?[16,32]:[60,90]};"
    "function tempStep(d){let b=tempBounds(),v=Math.max(b[0],Math.min(b[1],(parseInt($('temp').textContent)||b[0])+d));$('temp').textContent=v;queue('temp '+v+' '+unit)};"
    "function changed(k){modeChanged();if(k=='mode')queue('mode '+$('mode').value);else if(k=='fan')queue('fan '+$('fan').value)};"
    "function show(j){if(j.ok){lastUpdate=Date.now();updateAge()}else{$('status').textContent=j.error||'failed'};if(!j.state)return;let s=j.state;unit=s.unit.toLowerCase();$('temp').textContent=s.temperature;$('mode').value=s.mode;$('fan').value=s.fan;setToggle('vswing',s.vswing);setToggle('hswing',s.hswing);setSeg('quiet',s.quiet?'on':'off');setSensor(s.sensor);$('summaryTemp').textContent=tempModes[s.mode]?s.temperature+' '+deg+s.unit:s.mode;$('summaryMeta').textContent=s.mode+' - fan '+s.fan;setPower(s.power);modeChanged()};"
    "async function power(p){await cmd(p+(tempModes[$('mode').value]?' '+$('temp').textContent+' '+unit:''))};"
    "setInterval(updateAge,30000);fetch('/send?cmd=status').then(r=>r.json()).then(j=>{show(j);ready=1}).catch(()=>{ready=1});"
    "</script></body></html>";

static const char s_settings_page[] =
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Settings</title><style>"
    "body{font:15px system-ui,-apple-system,Segoe UI,sans-serif;margin:0;background:#f7f8fa;color:#171b22}"
    "main{max-width:760px;margin:auto;padding:18px}header{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}"
    "h1{font-size:22px;margin:0}a{color:#1769e0;text-decoration:none}section{background:white;border:1px solid #d9dee5;border-radius:8px;padding:14px;margin:12px 0}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}.stack{grid-template-columns:1fr}label{display:grid;gap:5px;color:#46515f;font-size:13px;min-width:0}"
    "select,input{font:inherit;width:100%;box-sizing:border-box;padding:10px;border:1px solid #c6ccd4;border-radius:6px;background:white}button{font:inherit;padding:10px 12px;border:0;border-radius:6px;background:#1769e0;color:white}"
    "button.secondary{background:#3d4856}.row{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}.slot{padding:8px 0;border-top:1px solid #edf0f3}.slot:first-child{border-top:0}.inlineStatus,#status{min-height:22px;color:#263241;margin:8px 0 0}"
    ".cardHead{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:10px}.cardHead h2{margin:0}.toggle{display:flex;justify-content:flex-end;gap:6px}.toggle button{background:#eef2f7;color:#1c2633}.toggle button.active{background:#1769e0;color:white}"
    ".wifiSummary,.homekitSummary{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:12px}.metric{background:#f7f8fa;border:1px solid #edf0f3;border-radius:6px;padding:10px}.metric b{display:block;margin-top:4px}.code{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;letter-spacing:.04em}.bars{display:flex;align-items:end;gap:3px;height:18px;margin-top:4px}.bars span{width:6px;background:#c6ccd4;border-radius:2px}.bars span:nth-child(1){height:6px}.bars span:nth-child(2){height:10px}.bars span:nth-child(3){height:14px}.bars span:nth-child(4){height:18px}.bars.s1 span:nth-child(-n+1),.bars.s2 span:nth-child(-n+2),.bars.s3 span:nth-child(-n+3),.bars.s4 span:nth-child(-n+4){background:#1769e0}.savedTitle{font-weight:600;margin:12px 0 4px}.slot{display:flex;align-items:center;justify-content:space-between;gap:8px}.slotName{overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.slotActions{display:flex;gap:6px;flex-shrink:0}.slotActions button{padding:8px 10px}"
    ".toast{position:fixed;left:50%;bottom:18px;transform:translate(-50%,18px);opacity:0;background:#171b22;color:white;border-radius:6px;padding:10px 14px;box-shadow:0 8px 24px rgba(0,0,0,.18);pointer-events:none;transition:opacity .18s,transform .18s}.toast.show{opacity:1;transform:translate(-50%,0)}"
    "</style></head><body><main><header><h1>Settings</h1><a href=/>AC Controls</a></header>"
    "<section><h2>Display</h2><div class=grid>"
    "<label>Temperature Unit<select id=unitSetting><option value=fahrenheit>&deg;F</option><option value=celsius>&deg;C</option></select></label>"
    "</div><div class=row><button type=button onclick=saveUnit()>Save Unit</button></div><p class=inlineStatus id=displayStatus></p></section>"
    "<section><h2>IR Blaster</h2><div class=grid>"
    "<label>Polarity<select id=polarity><option value=normal>Normal</option><option value=invert>Invert</option></select></label>"
    "<label>Timing<select id=timing><option value=nominal>Nominal</option><option value=captured>Captured</option></select></label>"
    "<label>Repeat<input id=repeat type=number min=1 max=10 value=1></label>"
    "<label>Gap (ms)<input id=gap type=number min=0 max=1000 value=80></label>"
    "</div><div class=row><button type=button onclick=saveIr()>Apply IR Settings</button></div><p class=inlineStatus id=irStatus></p></section>"
    "<section><h2>Wi-Fi</h2><div id=wifiStatus></div><div class=savedTitle>Hostname</div><form id=hostnameForm><div class='grid stack'>"
    "<label>Hostname<input id=hostname name=hostname maxlength=32 required></label>"
    "</div><div class=row><button>Save Hostname</button></div></form><div class=savedTitle>Saved Networks</div><div id=wifiSlots></div><div class=savedTitle>Add Network</div><form id=wifiForm><div class='grid stack'>"
    "<label>SSID<input id=ssid name=ssid maxlength=32 required></label>"
    "<label>Password<input id=password name=password maxlength=64 type=password></label>"
    "</div><div class=row><button>Save Wi-Fi</button><button class=secondary type=button onclick=\"document.getElementById('ssid').value='';document.getElementById('password').value=''\">Clear</button></div></form></section>"
    "<section><div class=cardHead><h2>HomeKit</h2><div class=toggle id=homekitToggle><button type=button data-v=1>On</button><button type=button data-v=0>Off</button></div></div>"
    "<div id=homekitDetails></div><div class=row><button type=button onclick=saveHomeKit()>Save HomeKit</button></div><p class=inlineStatus id=homekitStatus></p></section>"
    "<section><form id=mqttForm><div class=cardHead><h2>MQTT</h2><div class=toggle id=mqttToggle><button type=button data-v=1>On</button><button type=button data-v=0>Off</button></div></div><div class='grid stack'>"
    "<label>Broker IP/URI<input id=mqttBroker name=broker maxlength=127 placeholder='mqtt://192.168.1.20:1883'></label>"
    "<label>Subscribe Topic<input id=mqttSub name=subscribe maxlength=95 placeholder='esp-arc452a21/command'></label>"
    "<label>Publish Topic<input id=mqttPub name=publish maxlength=95 placeholder='esp-arc452a21/status'></label>"
    "</div><div class=row><button>Save MQTT</button><button class=secondary type=button onclick=\"document.getElementById('mqttBroker').value='';document.getElementById('mqttSub').value='';document.getElementById('mqttPub').value=''\">Clear</button></div></form></section>"
    "<p id=status></p></main><div id=toast class=toast role=status aria-live=polite>Saved</div><script>"
    "const $=id=>document.getElementById(id);async function post(path,body,type='application/x-www-form-urlencoded'){let r=await fetch(path,{method:'POST',headers:{'Content-Type':type},body});let t=await r.text();let j;try{j=JSON.parse(t)}catch(e){j={ok:r.ok,message:t}}if(!r.ok)j.ok=false;return j}"
    "let toastTimer=0;function toast(){let t=$('toast');t.textContent='Saved';t.classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>t.classList.remove('show'),1800)}"
    "function msg(id,j,toastOk=false){$(id).textContent=j.ok?(j.message||'ok'):(j.error||j.message||'failed');if(toastOk&&j.ok)toast()}"
    "function setToggle(id,v){document.querySelectorAll('#'+id+' button').forEach(b=>b.classList.toggle('active',b.dataset.v==String(v)))}"
    "function toggleValue(id){let b=document.querySelector('#'+id+' button.active');return b?b.dataset.v:'0'}"
    "['homekitToggle','mqttToggle'].forEach(id=>$(id).onclick=e=>{if(e.target.dataset.v)setToggle(id,e.target.dataset.v)});"
    "async function saveUnit(){msg('displayStatus',{ok:true,message:'Saving...'});msg('displayStatus',await post('/command','unit '+$('unitSetting').value,'text/plain'),true)}"
    "async function saveIr(){msg('irStatus',{ok:true,message:'Applying...'});msg('irStatus',await post('/settings/ir','polarity='+encodeURIComponent($('polarity').value)+'&timing='+encodeURIComponent($('timing').value)+'&repeat='+encodeURIComponent($('repeat').value)+'&gap='+encodeURIComponent($('gap').value)),true)}"
    "async function saveHomeKit(){msg('homekitStatus',{ok:true,message:'Saving...'});msg('homekitStatus',await post('/homekit','enabled='+encodeURIComponent(toggleValue('homekitToggle'))),true)}"
    "$('hostnameForm').onsubmit=async e=>{e.preventDefault();msg('status',await post('/wifi','action=hostname&hostname='+encodeURIComponent($('hostname').value)),true)};"
    "$('wifiForm').onsubmit=async e=>{e.preventDefault();msg('status',await post('/wifi','ssid='+encodeURIComponent($('ssid').value)+'&password='+encodeURIComponent($('password').value)),true);setTimeout(loadWifi,400)};"
    "$('mqttForm').onsubmit=async e=>{e.preventDefault();msg('status',await post('/mqtt','enabled='+encodeURIComponent(toggleValue('mqttToggle'))+'&broker='+encodeURIComponent($('mqttBroker').value)+'&subscribe='+encodeURIComponent($('mqttSub').value)+'&publish='+encodeURIComponent($('mqttPub').value)),true)};"
    "let wifiList=[];function esc(s){return s.replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',\"'\":'&#39;'}[c]))}"
    "$('wifiSlots').onclick=async e=>{let i=e.target.dataset.i;if(i===undefined)return;if(e.target.dataset.action=='delete'){if(!confirm('Delete saved Wi-Fi network '+wifiList[i].ssid+'?'))return;msg('status',await post('/wifi','action=delete&slot='+encodeURIComponent(i)),true);loadWifi();return}$('ssid').value=wifiList[i].ssid;$('password').value=''};"
    "function signalLevel(r){return r>=-55?4:r>=-67?3:r>=-75?2:r?1:0}"
    "function bars(l){return '<div class=\"bars s'+l+'\"><span></span><span></span><span></span><span></span></div>'}"
    "async function loadWifi(){let j=await fetch('/wifi').then(r=>r.json()).catch(()=>({slots:[]}));let l=signalLevel(j.rssi||0);$('hostname').value=j.hostname||'';$('wifiStatus').innerHTML='<div class=wifiSummary><div class=metric>Network<b>'+(j.connected?esc(j.ssid||'Connected'):'Not connected')+'</b></div><div class=metric>Hostname<b>'+esc(j.hostname||'--')+'</b></div><div class=metric>Signal'+bars(l)+'<b>'+(j.connected?(j.rssi+' dBm'):'--')+'</b></div><div class=metric>LAN IP<b>'+(j.ip||'--')+'</b></div></div>';wifiList=j.slots||[];$('wifiSlots').innerHTML=wifiList.length?wifiList.map((x,i)=>'<div class=slot><span class=slotName>'+esc(x.ssid)+'</span><span class=slotActions><button type=button data-i='+i+'>Update</button><button class=secondary type=button data-action=delete data-i='+i+'>Delete</button></span></div>').join(''):'<div class=slot>No saved networks</div>'}"
    "function homeKitStatus(j){if(!j.available)return'Not compiled';if(!j.enabled)return'Off';if(!j.started)return'Restarting to start';if(j.pairing)return'Pairing';if(j.connected_controller_count>0)return'Connected';if(j.paired_controller_count>0)return'Paired';if(j.pairing_timed_out)return'Pairing timed out';return'Ready to pair'}"
    "async function loadHomeKit(){let j=await fetch('/homekit').then(r=>r.json()).catch(()=>({enabled:true}));setToggle('homekitToggle',j.enabled?1:0);$('homekitDetails').innerHTML=j.enabled?'<div class=homekitSummary><div class=metric>Status<b>'+esc(homeKitStatus(j))+'</b></div><div class=metric>Pairing Code<b class=code>'+esc(j.setup_code||'--')+'</b></div><div class=metric>Controllers<b>'+((j.paired_controller_count||0)+' paired / '+(j.connected_controller_count||0)+' connected')+'</b></div><div class=metric>Setup ID<b class=code>'+esc(j.setup_id||'--')+'</b></div></div>':''}"
    "async function loadMqtt(){let j=await fetch('/mqtt').then(r=>r.json()).catch(()=>({}));setToggle('mqttToggle',j.enabled?1:0);$('mqttBroker').value=j.broker||'';$('mqttSub').value=j.subscribe_topic||'';$('mqttPub').value=j.publish_topic||''}"
    "fetch('/send?cmd=status').then(r=>r.json()).then(j=>{if(j.state)$('unitSetting').value=j.state.unit=='C'?'celsius':'fahrenheit';if(j.ir){$('polarity').value=j.ir.polarity||'normal';$('timing').value=j.ir.timing||'nominal';$('repeat').value=j.ir.repeat||1;$('gap').value=j.ir.gap_ms!==undefined?j.ir.gap_ms:80}}).catch(()=>{});loadWifi();loadHomeKit();loadMqtt();"
    "</script></body></html>";

static bool has_text(const char *value)
{
    return value != NULL && value[0] != '\0';
}

static void publish_mqtt_status(const char *payload)
{
    if (s_mqtt == NULL || !has_text(payload)) {
        return;
    }
    esp_mqtt_client_publish(s_mqtt, s_mqtt_status_topic, payload, 0, 1, 0);
}

static esp_err_t run_command(const char *command, char *response, size_t response_size)
{
    if (s_config.command_handler == NULL) {
        snprintf(response, response_size, "{\"ok\":false,\"error\":\"command handler unavailable\"}");
        return ESP_ERR_INVALID_STATE;
    }
    return s_config.command_handler(command, response, response_size);
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t send_html(httpd_req_t *req, const char *html)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    return httpd_resp_sendstr(req, html);
}

static esp_err_t health_get_handler(httpd_req_t *req)
{
    return send_json(req, "{\"ok\":true,\"device\":\"esp-arc452a21\"}");
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    return send_html(req, s_control_page);
}

static esp_err_t command_get_handler(httpd_req_t *req)
{
    char query[COMMAND_MAX_LEN + 16] = {};
    char command[COMMAND_MAX_LEN] = {};
    char response[RESPONSE_MAX_LEN] = {};

    esp_err_t err = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (err != ESP_OK || httpd_query_key_value(query, "cmd", command, sizeof(command)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"missing cmd query parameter\"}");
    }

    err = run_command(command, response, sizeof(response));
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    return send_json(req, response);
}

static esp_err_t command_post_handler(httpd_req_t *req)
{
    char command[COMMAND_MAX_LEN] = {};
    char response[RESPONSE_MAX_LEN] = {};
    size_t remaining = req->content_len;
    size_t offset = 0;

    if (remaining == 0 || remaining >= sizeof(command)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"command body must be 1..95 bytes\"}");
    }

    while (remaining > 0) {
        int received = httpd_req_recv(req, command + offset, remaining);
        if (received <= 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"failed to read command body\"}");
        }
        offset += (size_t)received;
        remaining -= (size_t)received;
    }
    command[offset] = '\0';

    esp_err_t err = run_command(command, response, sizeof(response));
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    return send_json(req, response);
}

static bool hex_value(char value, uint8_t *out)
{
    if (value >= '0' && value <= '9') {
        *out = (uint8_t)(value - '0');
        return true;
    }
    value = (char)tolower((unsigned char)value);
    if (value >= 'a' && value <= 'f') {
        *out = (uint8_t)(value - 'a' + 10);
        return true;
    }
    return false;
}

static void url_decode(char *value)
{
    char *read = value;
    char *write = value;

    while (*read != '\0') {
        if (*read == '+') {
            *write++ = ' ';
            ++read;
        } else if (*read == '%' && isxdigit((unsigned char)read[1]) &&
                   isxdigit((unsigned char)read[2])) {
            uint8_t high = 0;
            uint8_t low = 0;
            hex_value(read[1], &high);
            hex_value(read[2], &low);
            *write++ = (char)((high << 4) | low);
            read += 3;
        } else {
            *write++ = *read++;
        }
    }
    *write = '\0';
}

static bool form_get_value(const char *form, const char *key, char *out, size_t out_size)
{
    size_t key_len = strlen(key);
    const char *field = form;

    while (field != NULL && *field != '\0') {
        const char *next = strchr(field, '&');
        size_t field_len = next == NULL ? strlen(field) : (size_t)(next - field);
        if (field_len > key_len && strncmp(field, key, key_len) == 0 && field[key_len] == '=') {
            size_t value_len = field_len - key_len - 1;
            if (value_len >= out_size) {
                value_len = out_size - 1;
            }
            memcpy(out, field + key_len + 1, value_len);
            out[value_len] = '\0';
            url_decode(out);
            return true;
        }
        field = next == NULL ? NULL : next + 1;
    }
    return false;
}

static bool parse_enabled_value(const char *value, bool *enabled)
{
    if (strcmp(value, "1") == 0 || strcmp(value, "on") == 0 ||
        strcmp(value, "true") == 0 || strcmp(value, "enabled") == 0) {
        *enabled = true;
        return true;
    }
    if (strcmp(value, "0") == 0 || strcmp(value, "off") == 0 ||
        strcmp(value, "false") == 0 || strcmp(value, "disabled") == 0) {
        *enabled = false;
        return true;
    }
    return false;
}

static bool normalize_hostname(const char *input, char *out, size_t out_size)
{
    size_t len = 0;

    if (input == NULL || out_size == 0) {
        return false;
    }

    while (isspace((unsigned char)*input)) {
        ++input;
    }

    for (const unsigned char *ch = (const unsigned char *)input;
         *ch != '\0' && !isspace(*ch); ++ch) {
        if (len + 1 >= out_size || len >= DEVICE_HOSTNAME_MAX_LEN) {
            return false;
        }
        if (isalnum(*ch)) {
            out[len++] = (char)tolower(*ch);
        } else if (*ch == '-') {
            out[len++] = '-';
        } else {
            return false;
        }
    }

    out[len] = '\0';
    return len > 0 && out[0] != '-' && out[len - 1] != '-';
}

static esp_err_t read_form_body(httpd_req_t *req, char *form, size_t form_size)
{
    size_t remaining = req->content_len;
    size_t offset = 0;

    if (remaining == 0 || remaining >= form_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    while (remaining > 0) {
        int received = httpd_req_recv(req, form + offset, remaining);
        if (received <= 0) {
            return ESP_FAIL;
        }
        offset += (size_t)received;
        remaining -= (size_t)received;
    }
    form[offset] = '\0';
    return ESP_OK;
}

static void json_escape_string(const char *value, char *out, size_t out_size)
{
    size_t offset = 0;
    for (const unsigned char *ch = (const unsigned char *)value;
         *ch != '\0' && offset + 1 < out_size; ++ch) {
        if (*ch == '"' || *ch == '\\') {
            if (offset + 2 >= out_size) {
                break;
            }
            out[offset++] = '\\';
            out[offset++] = (char)*ch;
        } else if (*ch < 0x20) {
            if (offset + 6 >= out_size) {
                break;
            }
            offset += snprintf(out + offset, out_size - offset, "\\u%04x", *ch);
        } else {
            out[offset++] = (char)*ch;
        }
    }
    out[offset] = '\0';
}

static esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    wifi_credential_t credentials[WIFI_CREDENTIAL_SLOTS] = {};
    uint8_t count = (uint8_t)s_wifi_credential_count;
    if (count > WIFI_CREDENTIAL_SLOTS) {
        count = WIFI_CREDENTIAL_SLOTS;
    }
    memcpy(credentials, s_wifi_credentials, sizeof(credentials));

    int slot = -1;
    for (uint8_t i = 0; i < count; ++i) {
        if (strcmp(credentials[i].ssid, ssid) == 0) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        if (count < WIFI_CREDENTIAL_SLOTS) {
            slot = count;
            ++count;
        } else {
            memmove(&credentials[0], &credentials[1],
                    (WIFI_CREDENTIAL_SLOTS - 1) * sizeof(credentials[0]));
            slot = WIFI_CREDENTIAL_SLOTS - 1;
        }
    }

    strlcpy(credentials[slot].ssid, ssid, sizeof(credentials[slot].ssid));
    strlcpy(credentials[slot].password, password == NULL ? "" : password,
            sizeof(credentials[slot].password));

    nvs_handle_t nvs = 0;
    ESP_RETURN_ON_ERROR(nvs_open("wifi_cfg", NVS_READWRITE, &nvs),
                        TAG, "failed to open WiFi NVS namespace");

    esp_err_t err = ESP_OK;
    for (uint8_t i = 0; i < WIFI_CREDENTIAL_SLOTS && err == ESP_OK; ++i) {
        char ssid_key[8] = {};
        char password_key[8] = {};
        snprintf(ssid_key, sizeof(ssid_key), "ssid%u", (unsigned)i);
        snprintf(password_key, sizeof(password_key), "pass%u", (unsigned)i);
        if (i < count) {
            err = nvs_set_str(nvs, ssid_key, credentials[i].ssid);
            if (err == ESP_OK) {
                err = nvs_set_str(nvs, password_key, credentials[i].password);
            }
        } else {
            esp_err_t erase_err = nvs_erase_key(nvs, ssid_key);
            if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
                err = erase_err;
            }
            erase_err = nvs_erase_key(nvs, password_key);
            if (err == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
                err = erase_err;
            }
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "count", count);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "next", count < WIFI_CREDENTIAL_SLOTS ? count : 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t delete_wifi_credential(uint8_t slot)
{
    if (slot >= s_wifi_credential_count) {
        return ESP_ERR_INVALID_ARG;
    }

    if (slot + 1 < s_wifi_credential_count) {
        memmove(&s_wifi_credentials[slot], &s_wifi_credentials[slot + 1],
                (s_wifi_credential_count - slot - 1) * sizeof(s_wifi_credentials[0]));
    }
    --s_wifi_credential_count;
    memset(&s_wifi_credentials[s_wifi_credential_count], 0, sizeof(s_wifi_credentials[0]));
    if (s_current_wifi_credential >= s_wifi_credential_count) {
        s_current_wifi_credential = 0;
    }

    nvs_handle_t nvs = 0;
    ESP_RETURN_ON_ERROR(nvs_open("wifi_cfg", NVS_READWRITE, &nvs),
                        TAG, "failed to open WiFi NVS namespace");

    esp_err_t err = ESP_OK;
    for (uint8_t i = 0; i < WIFI_CREDENTIAL_SLOTS && err == ESP_OK; ++i) {
        char ssid_key[8] = {};
        char password_key[8] = {};
        snprintf(ssid_key, sizeof(ssid_key), "ssid%u", (unsigned)i);
        snprintf(password_key, sizeof(password_key), "pass%u", (unsigned)i);
        if (i < s_wifi_credential_count) {
            err = nvs_set_str(nvs, ssid_key, s_wifi_credentials[i].ssid);
            if (err == ESP_OK) {
                err = nvs_set_str(nvs, password_key, s_wifi_credentials[i].password);
            }
        } else {
            esp_err_t erase_err = nvs_erase_key(nvs, ssid_key);
            if (erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
                err = erase_err;
            }
            erase_err = nvs_erase_key(nvs, password_key);
            if (err == ESP_OK && erase_err != ESP_OK && erase_err != ESP_ERR_NVS_NOT_FOUND) {
                err = erase_err;
            }
        }
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "count", (uint8_t)s_wifi_credential_count);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "next", s_wifi_credential_count < WIFI_CREDENTIAL_SLOTS
                                      ? (uint8_t)s_wifi_credential_count
                                      : 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static void load_device_hostname_setting(void)
{
    char hostname[DEVICE_HOSTNAME_MAX_LEN + 1] = DEVICE_HOSTNAME_DEFAULT;
    nvs_handle_t nvs = 0;

    strlcpy(s_device_hostname, DEVICE_HOSTNAME_DEFAULT, sizeof(s_device_hostname));
    if (nvs_open("app_cfg", NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }

    size_t len = sizeof(hostname);
    if (nvs_get_str(nvs, "hostname", hostname, &len) == ESP_OK) {
        char normalized[DEVICE_HOSTNAME_MAX_LEN + 1] = {};
        if (normalize_hostname(hostname, normalized, sizeof(normalized))) {
            strlcpy(s_device_hostname, normalized, sizeof(s_device_hostname));
        }
    }
    nvs_close(nvs);
}

static esp_err_t save_device_hostname_setting(const char *hostname)
{
    char normalized[DEVICE_HOSTNAME_MAX_LEN + 1] = {};
    nvs_handle_t nvs = 0;

    if (!normalize_hostname(hostname, normalized, sizeof(normalized))) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(nvs_open("app_cfg", NVS_READWRITE, &nvs),
                        TAG, "failed to open app settings NVS namespace");

    esp_err_t err = nvs_set_str(nvs, "hostname", normalized);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (err == ESP_OK) {
        strlcpy(s_device_hostname, normalized, sizeof(s_device_hostname));
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t wifi_get_handler(httpd_req_t *req)
{
    char json[1200] = {};
    size_t offset = 0;
    wifi_ap_record_t ap_info = {};
    esp_netif_ip_info_t ip_info = {};
    bool connected = s_sta_connected && esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK;
    bool has_ip = connected && s_sta_netif != NULL &&
                  esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK;
    char connected_ssid[WIFI_SSID_MAX_LEN * 6 + 1] = {};
    char hostname[DEVICE_HOSTNAME_MAX_LEN * 6 + 1] = {};
    char ip_addr[16] = {};

    if (connected) {
        json_escape_string((const char *)ap_info.ssid, connected_ssid, sizeof(connected_ssid));
    }
    json_escape_string(s_device_hostname, hostname, sizeof(hostname));
    if (has_ip) {
        snprintf(ip_addr, sizeof(ip_addr), IPSTR, IP2STR(&ip_info.ip));
    }

    offset += snprintf(json + offset, sizeof(json) - offset,
                       "{\"ok\":true,\"connected\":%s,\"ssid\":\"%s\","
                       "\"rssi\":%d,\"ip\":\"%s\",\"hostname\":\"%s\",\"slots\":[",
                       connected ? "true" : "false",
                       connected_ssid,
                       connected ? ap_info.rssi : 0,
                       ip_addr,
                       hostname);
    for (size_t i = 0; i < s_wifi_credential_count; ++i) {
        char escaped_ssid[WIFI_SSID_MAX_LEN * 6 + 1] = {};
        json_escape_string(s_wifi_credentials[i].ssid, escaped_ssid, sizeof(escaped_ssid));
        offset += snprintf(json + offset, sizeof(json) - offset,
                           "%s{\"slot\":%u,\"ssid\":\"%s\"}",
                           i == 0 ? "" : ",", (unsigned)i, escaped_ssid);
    }
    offset += snprintf(json + offset, sizeof(json) - offset, "]}");
    return send_json(req, json);
}

static esp_err_t wifi_post_handler(httpd_req_t *req)
{
    char form[WIFI_FORM_MAX_LEN] = {};
    char action[16] = {};
    char slot_value[8] = {};
    char hostname[DEVICE_HOSTNAME_MAX_LEN + 1] = {};
    char ssid[WIFI_SSID_MAX_LEN + 1] = {};
    char password[WIFI_PASSWORD_MAX_LEN + 1] = {};

    if (read_form_body(req, form, sizeof(form)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_html(req, "<html><body>WiFi form is too large.</body></html>");
    }

    if (form_get_value(form, "action", action, sizeof(action)) &&
        strcmp(action, "hostname") == 0) {
        if (!form_get_value(form, "hostname", hostname, sizeof(hostname)) ||
            !has_text(hostname)) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"hostname is required\"}");
        }

        esp_err_t err = save_device_hostname_setting(hostname);
        if (err == ESP_ERR_INVALID_ARG) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"hostname must use letters, numbers, or hyphens\"}");
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to save hostname: %s", esp_err_to_name(err));
            httpd_resp_set_status(req, "500 Internal Server Error");
            return send_json(req, "{\"ok\":false,\"error\":\"failed to save hostname\"}");
        }

        ESP_LOGI(TAG, "Saved hostname \"%s\"; restarting", s_device_hostname);
        esp_err_t send_err = send_json(req, "{\"ok\":true,\"message\":\"Hostname saved; restarting\"}");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return send_err;
    }

    if (strcmp(action, "delete") == 0) {
        if (!form_get_value(form, "slot", slot_value, sizeof(slot_value))) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"slot is required\"}");
        }
        char *end = NULL;
        long slot = strtol(slot_value, &end, 10);
        if (end == slot_value || *end != '\0' || slot < 0 ||
            slot >= (long)s_wifi_credential_count) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"invalid WiFi slot\"}");
        }

        char deleted_ssid[WIFI_SSID_MAX_LEN + 1] = {};
        strlcpy(deleted_ssid, s_wifi_credentials[slot].ssid, sizeof(deleted_ssid));
        esp_err_t err = delete_wifi_credential((uint8_t)slot);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete WiFi credential: %s", esp_err_to_name(err));
            httpd_resp_set_status(req, "500 Internal Server Error");
            return send_json(req, "{\"ok\":false,\"error\":\"failed to delete WiFi network\"}");
        }

        ESP_LOGI(TAG, "Deleted WiFi credentials for SSID \"%s\"", deleted_ssid);
        return send_json(req, "{\"ok\":true,\"message\":\"WiFi network deleted\"}");
    }

    if (!form_get_value(form, "ssid", ssid, sizeof(ssid)) || !has_text(ssid)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"SSID is required\"}");
    }
    form_get_value(form, "password", password, sizeof(password));

    esp_err_t err = save_wifi_credentials(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save WiFi credentials: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_json(req, "{\"ok\":false,\"error\":\"failed to save WiFi settings\"}");
    }

    ESP_LOGI(TAG, "Saved WiFi credentials for SSID \"%s\"; restarting", ssid);
    esp_err_t send_err = send_json(req, "{\"ok\":true,\"message\":\"WiFi saved; restarting\"}");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return send_err;
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    return send_html(req, s_settings_page);
}

static esp_err_t settings_ir_post_handler(httpd_req_t *req)
{
    char form[WIFI_FORM_MAX_LEN] = {};
    char polarity[16] = {};
    char timing[16] = {};
    char repeat[8] = {};
    char gap[8] = {};
    char command[COMMAND_MAX_LEN] = {};
    char response[RESPONSE_MAX_LEN] = {};

    if (read_form_body(req, form, sizeof(form)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"settings form is too large\"}");
    }

    if (form_get_value(form, "polarity", polarity, sizeof(polarity))) {
        snprintf(command, sizeof(command), "polarity %s", polarity);
        esp_err_t err = run_command(command, response, sizeof(response));
        if (err != ESP_OK) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, response);
        }
    }

    if (form_get_value(form, "timing", timing, sizeof(timing))) {
        snprintf(command, sizeof(command), "timing %s", timing);
        esp_err_t err = run_command(command, response, sizeof(response));
        if (err != ESP_OK) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, response);
        }
    }

    if (form_get_value(form, "repeat", repeat, sizeof(repeat))) {
        if (!form_get_value(form, "gap", gap, sizeof(gap))) {
            strlcpy(gap, "80", sizeof(gap));
        }
        snprintf(command, sizeof(command), "repeat %s %s", repeat, gap);
        esp_err_t err = run_command(command, response, sizeof(response));
        if (err != ESP_OK) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, response);
        }
    }

    return send_json(req, "{\"ok\":true,\"message\":\"IR settings applied\"}");
}

static bool load_homekit_enabled_setting(void)
{
    bool enabled = CONFIG_ESP_ARC452A21_HOMEKIT_ENABLE;
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("app_cfg", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return enabled;
    }

    uint8_t saved = enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "hk_en", &saved) == ESP_OK) {
        enabled = saved != 0;
    }
    nvs_close(nvs);
    return enabled;
}

static esp_err_t save_homekit_enabled_setting(bool enabled)
{
    nvs_handle_t nvs = 0;
    ESP_RETURN_ON_ERROR(nvs_open("app_cfg", NVS_READWRITE, &nvs),
                        TAG, "failed to open app settings NVS namespace");

    esp_err_t err = nvs_set_u8(nvs, "hk_en", enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t homekit_get_handler(httpd_req_t *req)
{
    homekit_accessory_status_t status = {};
    char setup_code[sizeof(status.setup_code) * 6 + 1] = {};
    char setup_id[sizeof(status.setup_id) * 6 + 1] = {};
    char setup_payload[sizeof(status.setup_payload) * 6 + 1] = {};
    char response[640] = {};

    homekit_accessory_get_status(&status);
    json_escape_string(status.setup_code, setup_code, sizeof(setup_code));
    json_escape_string(status.setup_id, setup_id, sizeof(setup_id));
    json_escape_string(status.setup_payload, setup_payload, sizeof(setup_payload));

    snprintf(response, sizeof(response),
             "{\"ok\":true,\"enabled\":%s,\"available\":%s,"
             "\"started\":%s,\"pairing\":%s,\"pairing_timed_out\":%s,"
             "\"paired_controller_count\":%d,\"connected_controller_count\":%d,"
             "\"setup_code\":\"%s\",\"setup_id\":\"%s\",\"setup_payload\":\"%s\","
             "\"hap_port\":%d}",
             load_homekit_enabled_setting() ? "true" : "false",
             status.compiled ? "true" : "false",
             status.started ? "true" : "false",
             status.pairing ? "true" : "false",
             status.pairing_timed_out ? "true" : "false",
             status.paired_controller_count,
             status.connected_controller_count,
             setup_code,
             setup_id,
             setup_payload,
             CONFIG_HAP_HTTP_SERVER_PORT);
    return send_json(req, response);
}

static esp_err_t homekit_post_handler(httpd_req_t *req)
{
    char form[WIFI_FORM_MAX_LEN] = {};
    char enabled_value[16] = {};
    bool enabled = false;

    if (read_form_body(req, form, sizeof(form)) != ESP_OK ||
        !form_get_value(form, "enabled", enabled_value, sizeof(enabled_value)) ||
        !parse_enabled_value(enabled_value, &enabled)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"HomeKit enabled value is required\"}");
    }

    if (enabled && !CONFIG_ESP_ARC452A21_HOMEKIT_ENABLE) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"HomeKit is not compiled into this firmware\"}");
    }

    esp_err_t err = save_homekit_enabled_setting(enabled);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save HomeKit setting: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_json(req, "{\"ok\":false,\"error\":\"failed to save HomeKit setting\"}");
    }

    ESP_LOGI(TAG, "Saved HomeKit setting; restarting");
    esp_err_t send_err = send_json(req, "{\"ok\":true,\"message\":\"HomeKit saved; restarting\"}");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return send_err;
}

static void normalize_mqtt_uri(const char *input, char *out, size_t out_size)
{
    if (!has_text(input)) {
        out[0] = '\0';
        return;
    }

    if (strstr(input, "://") != NULL) {
        strlcpy(out, input, out_size);
        return;
    }

    if (strchr(input, ':') != NULL) {
        snprintf(out, out_size, "mqtt://%s", input);
    } else {
        snprintf(out, out_size, "mqtt://%s:1883", input);
    }
}

static bool is_plain_mqtt_setting(const char *value)
{
    for (const char *ch = value; *ch != '\0'; ++ch) {
        if ((unsigned char)*ch < 0x21 || *ch == '"' || *ch == '\\') {
            return false;
        }
    }
    return true;
}

static void load_mqtt_settings(void)
{
    const char *prefix = has_text(s_config.mqtt_topic_prefix)
                             ? s_config.mqtt_topic_prefix
                             : "esp-arc452a21";

    strlcpy(s_mqtt_uri, s_config.mqtt_uri == NULL ? "" : s_config.mqtt_uri,
            sizeof(s_mqtt_uri));
    s_mqtt_enabled = has_text(s_mqtt_uri);
    snprintf(s_mqtt_command_topic, sizeof(s_mqtt_command_topic), "%s/command", prefix);
    snprintf(s_mqtt_status_topic, sizeof(s_mqtt_status_topic), "%s/status", prefix);
    snprintf(s_mqtt_availability_topic, sizeof(s_mqtt_availability_topic),
             "%s/availability", prefix);

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("mqtt_cfg", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }

    size_t len = sizeof(s_mqtt_uri);
    nvs_get_str(nvs, "uri", s_mqtt_uri, &len);
    len = sizeof(s_mqtt_command_topic);
    nvs_get_str(nvs, "sub", s_mqtt_command_topic, &len);
    len = sizeof(s_mqtt_status_topic);
    nvs_get_str(nvs, "pub", s_mqtt_status_topic, &len);
    uint8_t enabled = s_mqtt_enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "enabled", &enabled) == ESP_OK) {
        s_mqtt_enabled = enabled != 0;
    }
    nvs_close(nvs);
}

static esp_err_t save_mqtt_settings(bool enabled,
                                    const char *broker,
                                    const char *subscribe_topic,
                                    const char *publish_topic)
{
    char uri[MQTT_URI_MAX_LEN] = {};
    normalize_mqtt_uri(broker, uri, sizeof(uri));

    nvs_handle_t nvs = 0;
    ESP_RETURN_ON_ERROR(nvs_open("mqtt_cfg", NVS_READWRITE, &nvs),
                        TAG, "failed to open MQTT NVS namespace");

    esp_err_t err = nvs_set_u8(nvs, "enabled", enabled ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "uri", uri);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "sub", subscribe_topic);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(nvs, "pub", publish_topic);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t mqtt_get_handler(httpd_req_t *req)
{
    char broker[MQTT_URI_MAX_LEN + 1] = {};
    char subscribe_topic[MQTT_TOPIC_MAX_LEN + 1] = {};
    char publish_topic[MQTT_TOPIC_MAX_LEN + 1] = {};
    char response[512] = {};

    json_escape_string(s_mqtt_uri, broker, sizeof(broker));
    json_escape_string(s_mqtt_command_topic, subscribe_topic, sizeof(subscribe_topic));
    json_escape_string(s_mqtt_status_topic, publish_topic, sizeof(publish_topic));
    snprintf(response, sizeof(response),
             "{\"ok\":true,\"enabled\":%s,\"broker\":\"%s\",\"subscribe_topic\":\"%s\","
             "\"publish_topic\":\"%s\"}",
             s_mqtt_enabled ? "true" : "false",
             broker, subscribe_topic, publish_topic);
    return send_json(req, response);
}

static esp_err_t mqtt_post_handler(httpd_req_t *req)
{
    char form[WIFI_FORM_MAX_LEN] = {};
    char broker[MQTT_URI_MAX_LEN] = {};
    char subscribe_topic[MQTT_TOPIC_MAX_LEN] = {};
    char publish_topic[MQTT_TOPIC_MAX_LEN] = {};
    char enabled_value[16] = {};
    bool enabled = true;
    bool enabled_was_set = false;

    if (read_form_body(req, form, sizeof(form)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"MQTT form is too large\"}");
    }

    if (form_get_value(form, "enabled", enabled_value, sizeof(enabled_value))) {
        if (!parse_enabled_value(enabled_value, &enabled)) {
            httpd_resp_set_status(req, "400 Bad Request");
            return send_json(req, "{\"ok\":false,\"error\":\"invalid MQTT enabled value\"}");
        }
        enabled_was_set = true;
    }
    form_get_value(form, "broker", broker, sizeof(broker));
    form_get_value(form, "subscribe", subscribe_topic, sizeof(subscribe_topic));
    form_get_value(form, "publish", publish_topic, sizeof(publish_topic));
    if (!enabled_was_set) {
        enabled = has_text(broker);
    }

    if ((has_text(broker) && !is_plain_mqtt_setting(broker)) ||
        (has_text(subscribe_topic) && !is_plain_mqtt_setting(subscribe_topic)) ||
        (has_text(publish_topic) && !is_plain_mqtt_setting(publish_topic))) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"MQTT values cannot contain spaces or quotes\"}");
    }

    if (enabled && !has_text(broker)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"broker is required when MQTT is on\"}");
    }

    if (has_text(broker) && (!has_text(subscribe_topic) || !has_text(publish_topic))) {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_json(req, "{\"ok\":false,\"error\":\"subscribe and publish topics are required\"}");
    }

    if (!has_text(subscribe_topic)) {
        strlcpy(subscribe_topic, s_mqtt_command_topic, sizeof(subscribe_topic));
    }
    if (!has_text(publish_topic)) {
        strlcpy(publish_topic, s_mqtt_status_topic, sizeof(publish_topic));
    }

    esp_err_t err = save_mqtt_settings(enabled, broker, subscribe_topic, publish_topic);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save MQTT settings: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return send_json(req, "{\"ok\":false,\"error\":\"failed to save MQTT settings\"}");
    }

    ESP_LOGI(TAG, "Saved MQTT settings; restarting");
    esp_err_t send_err = send_json(req, "{\"ok\":true,\"message\":\"MQTT saved; restarting\"}");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return send_err;
}

static esp_err_t start_http_server(void)
{
    if (s_httpd != NULL) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = HTTP_URI_HANDLER_SLOTS;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_RETURN_ON_ERROR(httpd_start(&s_httpd, &config), TAG, "failed to start HTTP server");

    const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    const httpd_uri_t health_uri = {
        .uri = "/health",
        .method = HTTP_GET,
        .handler = health_get_handler,
    };
    const httpd_uri_t settings_uri = {
        .uri = "/settings",
        .method = HTTP_GET,
        .handler = settings_get_handler,
    };
    const httpd_uri_t send_uri = {
        .uri = "/send",
        .method = HTTP_GET,
        .handler = command_get_handler,
    };
    const httpd_uri_t command_uri = {
        .uri = "/command",
        .method = HTTP_POST,
        .handler = command_post_handler,
    };
    const httpd_uri_t wifi_get_uri = {
        .uri = "/wifi",
        .method = HTTP_GET,
        .handler = wifi_get_handler,
    };
    const httpd_uri_t wifi_uri = {
        .uri = "/wifi",
        .method = HTTP_POST,
        .handler = wifi_post_handler,
    };
    const httpd_uri_t mqtt_get_uri = {
        .uri = "/mqtt",
        .method = HTTP_GET,
        .handler = mqtt_get_handler,
    };
    const httpd_uri_t mqtt_uri = {
        .uri = "/mqtt",
        .method = HTTP_POST,
        .handler = mqtt_post_handler,
    };
    const httpd_uri_t homekit_get_uri = {
        .uri = "/homekit",
        .method = HTTP_GET,
        .handler = homekit_get_handler,
    };
    const httpd_uri_t homekit_uri = {
        .uri = "/homekit",
        .method = HTTP_POST,
        .handler = homekit_post_handler,
    };
    const httpd_uri_t settings_ir_uri = {
        .uri = "/settings/ir",
        .method = HTTP_POST,
        .handler = settings_ir_post_handler,
    };
    const httpd_uri_t captive_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = settings_get_handler,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &root_uri),
                        TAG, "failed to register /");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &health_uri),
                        TAG, "failed to register /health");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &settings_uri),
                        TAG, "failed to register /settings");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &send_uri),
                        TAG, "failed to register /send");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &command_uri),
                        TAG, "failed to register /command");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &wifi_get_uri),
                        TAG, "failed to register GET /wifi");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &wifi_uri),
                        TAG, "failed to register POST /wifi");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &mqtt_get_uri),
                        TAG, "failed to register GET /mqtt");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &mqtt_uri),
                        TAG, "failed to register POST /mqtt");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &homekit_get_uri),
                        TAG, "failed to register GET /homekit");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &homekit_uri),
                        TAG, "failed to register POST /homekit");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &settings_ir_uri),
                        TAG, "failed to register /settings/ir");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_httpd, &captive_uri),
                        TAG, "failed to register captive portal wildcard");

    ESP_LOGI(TAG, "HTTP remote control ready: /, /settings, /health, /send?cmd=72");
    return ESP_OK;
}

static bool mqtt_topic_matches(const esp_mqtt_event_handle_t event, const char *topic)
{
    size_t topic_len = strlen(topic);
    return event->topic_len == topic_len && strncmp(event->topic, topic, topic_len) == 0;
}

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        esp_mqtt_client_subscribe(event->client, s_mqtt_command_topic, 1);
        esp_mqtt_client_publish(event->client, s_mqtt_availability_topic, "online", 0, 1, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        break;
    case MQTT_EVENT_DATA: {
        if (!mqtt_topic_matches(event, s_mqtt_command_topic)) {
            break;
        }

        char command[COMMAND_MAX_LEN] = {};
        char response[RESPONSE_MAX_LEN] = {};
        size_t copy_len = event->data_len < sizeof(command) - 1
                              ? (size_t)event->data_len
                              : sizeof(command) - 1;
        memcpy(command, event->data, copy_len);
        command[copy_len] = '\0';

        esp_err_t err = run_command(command, response, sizeof(response));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "MQTT command failed: %s", esp_err_to_name(err));
        }
        publish_mqtt_status(response);
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

static esp_err_t start_mqtt_client(void)
{
    if (s_mqtt != NULL || !s_mqtt_enabled || !has_text(s_mqtt_uri)) {
        return ESP_OK;
    }

    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = s_mqtt_uri,
    };

    s_mqtt = esp_mqtt_client_init(&mqtt_config);
    ESP_RETURN_ON_FALSE(s_mqtt != NULL, ESP_ERR_NO_MEM, TAG, "failed to create MQTT client");
    ESP_RETURN_ON_ERROR(esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID,
                                                       mqtt_event_handler, NULL),
                        TAG, "failed to register MQTT event handler");
    ESP_RETURN_ON_ERROR(esp_mqtt_client_start(s_mqtt), TAG, "failed to start MQTT client");
    ESP_LOGI(TAG, "MQTT remote control connecting to %s; subscribe %s publish %s",
             s_mqtt_uri, s_mqtt_command_topic, s_mqtt_status_topic);
    return ESP_OK;
}

static bool read_wifi_slot(nvs_handle_t nvs, uint8_t slot, wifi_credential_t *credential)
{
    char ssid_key[8] = {};
    char password_key[8] = {};
    snprintf(ssid_key, sizeof(ssid_key), "ssid%u", (unsigned)slot);
    snprintf(password_key, sizeof(password_key), "pass%u", (unsigned)slot);

    size_t ssid_len = sizeof(credential->ssid);
    esp_err_t err = nvs_get_str(nvs, ssid_key, credential->ssid, &ssid_len);
    if (err != ESP_OK || !has_text(credential->ssid)) {
        return false;
    }

    size_t password_len = sizeof(credential->password);
    err = nvs_get_str(nvs, password_key, credential->password, &password_len);
    if (err != ESP_OK) {
        credential->password[0] = '\0';
    }
    return true;
}

static size_t load_saved_wifi_credentials(void)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open("wifi_cfg", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return 0;
    }

    uint8_t count = 0;
    nvs_get_u8(nvs, "count", &count);
    if (count > WIFI_CREDENTIAL_SLOTS) {
        count = WIFI_CREDENTIAL_SLOTS;
    }

    if (count == 0) {
        wifi_credential_t legacy = {};
        size_t ssid_len = sizeof(legacy.ssid);
        if (nvs_get_str(nvs, "ssid", legacy.ssid, &ssid_len) == ESP_OK &&
            has_text(legacy.ssid)) {
            size_t password_len = sizeof(legacy.password);
            if (nvs_get_str(nvs, "password", legacy.password, &password_len) != ESP_OK) {
                legacy.password[0] = '\0';
            }
            s_wifi_credentials[0] = legacy;
            nvs_close(nvs);
            return 1;
        }
    }

    size_t loaded = 0;
    for (uint8_t attempt = 0; attempt < count && loaded < WIFI_CREDENTIAL_SLOTS; ++attempt) {
        wifi_credential_t credential = {};
        if (read_wifi_slot(nvs, attempt, &credential)) {
            s_wifi_credentials[loaded++] = credential;
        }
    }

    nvs_close(nvs);
    return loaded;
}

static void load_station_credentials(void)
{
    memset(s_wifi_credentials, 0, sizeof(s_wifi_credentials));
    s_wifi_credential_count = load_saved_wifi_credentials();
    s_current_wifi_credential = 0;

    if (s_wifi_credential_count > 0) {
        ESP_LOGI(TAG, "Loaded %u saved WiFi network(s); first SSID \"%s\"",
                 (unsigned)s_wifi_credential_count, s_wifi_credentials[0].ssid);
        return;
    }

    if (has_text(s_config.wifi_ssid)) {
        strlcpy(s_wifi_credentials[0].ssid, s_config.wifi_ssid,
                sizeof(s_wifi_credentials[0].ssid));
        strlcpy(s_wifi_credentials[0].password, s_config.wifi_password,
                sizeof(s_wifi_credentials[0].password));
        s_wifi_credential_count = 1;
        ESP_LOGI(TAG, "Using build-configured WiFi SSID \"%s\"",
                 s_wifi_credentials[0].ssid);
        return;
    }
}

static esp_err_t apply_current_station_config(void)
{
    if (s_current_wifi_credential >= s_wifi_credential_count) {
        return ESP_ERR_NOT_FOUND;
    }

    const wifi_credential_t *credential = &s_wifi_credentials[s_current_wifi_credential];
    wifi_config_t wifi_config = {};
    strlcpy((char *)wifi_config.sta.ssid, credential->ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, credential->password,
            sizeof(wifi_config.sta.password));
    wifi_config.sta.failure_retry_cnt = 1;
    return esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
}

static void connect_next_station_or_start_portal(void)
{
    if (s_current_wifi_credential + 1 < s_wifi_credential_count) {
        ++s_current_wifi_credential;
        s_sta_retries = 0;
        ESP_LOGW(TAG, "Trying saved WiFi SSID \"%s\"",
                 s_wifi_credentials[s_current_wifi_credential].ssid);
        if (apply_current_station_config() == ESP_OK) {
            esp_wifi_connect();
            return;
        }
    }

    ESP_LOGW(TAG, "No known WiFi networks connected; starting setup portal");
    if (start_provisioning_ap() != ESP_OK) {
        ESP_LOGW(TAG, "Setup portal failed to start");
    }
}

static void dns_server_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGW(TAG, "Failed to create DNS socket: errno=%d", errno);
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGW(TAG, "Failed to bind DNS socket: errno=%d", errno);
        close(sock);
        s_dns_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Captive portal DNS server started");
    uint8_t request[DNS_PACKET_MAX_LEN];
    uint8_t response[DNS_PACKET_MAX_LEN];

    while (true) {
        struct sockaddr_in source_addr = {};
        socklen_t source_len = sizeof(source_addr);
        int len = recvfrom(sock, request, sizeof(request), 0,
                           (struct sockaddr *)&source_addr, &source_len);
        if (len < 17) {
            continue;
        }

        size_t question_end = 12;
        while (question_end < (size_t)len && request[question_end] != 0) {
            question_end += (size_t)request[question_end] + 1;
        }
        question_end += 5;
        if (question_end > (size_t)len || question_end + 16 > sizeof(response)) {
            continue;
        }

        esp_netif_ip_info_t ip_info = {};
        if (s_ap_netif == NULL || esp_netif_get_ip_info(s_ap_netif, &ip_info) != ESP_OK) {
            continue;
        }

        memcpy(response, request, question_end);
        response[2] = 0x81;
        response[3] = 0x80;
        response[4] = 0x00;
        response[5] = 0x01;
        response[6] = 0x00;
        response[7] = 0x01;
        response[8] = 0x00;
        response[9] = 0x00;
        response[10] = 0x00;
        response[11] = 0x00;

        size_t offset = question_end;
        response[offset++] = 0xC0;
        response[offset++] = 0x0C;
        response[offset++] = 0x00;
        response[offset++] = 0x01;
        response[offset++] = 0x00;
        response[offset++] = 0x01;
        response[offset++] = 0x00;
        response[offset++] = 0x00;
        response[offset++] = 0x00;
        response[offset++] = 0x00;
        response[offset++] = 0x00;
        response[offset++] = 0x04;
        memcpy(response + offset, &ip_info.ip.addr, 4);
        offset += 4;

        sendto(sock, response, offset, 0, (struct sockaddr *)&source_addr, source_len);
    }
}

static esp_err_t start_dns_server(void)
{
    if (s_dns_task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreate(dns_server_task, "captive_dns", 3072, NULL, 4, &s_dns_task);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t configure_setup_ap(void)
{
    wifi_config_t ap_config = {};
    strlcpy((char *)ap_config.ap.ssid, CONFIG_ESP_ARC452A21_SETUP_AP_SSID,
            sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = strlen(CONFIG_ESP_ARC452A21_SETUP_AP_SSID);
    ap_config.ap.channel = CONFIG_ESP_ARC452A21_SETUP_AP_CHANNEL;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    if (has_text(CONFIG_ESP_ARC452A21_SETUP_AP_PASSWORD)) {
        strlcpy((char *)ap_config.ap.password, CONFIG_ESP_ARC452A21_SETUP_AP_PASSWORD,
                sizeof(ap_config.ap.password));
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    return esp_wifi_set_config(WIFI_IF_AP, &ap_config);
}

static esp_err_t start_provisioning_services(void)
{
    if (s_ap_started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(start_http_server(), TAG, "failed to start setup HTTP server");
    ESP_RETURN_ON_ERROR(start_dns_server(), TAG, "failed to start captive DNS server");

    s_ap_started = true;
    ESP_LOGW(TAG, "Setup AP active: SSID \"%s\", open http://192.168.4.1/",
             CONFIG_ESP_ARC452A21_SETUP_AP_SSID);
    return ESP_OK;
}

static esp_err_t start_provisioning_ap(void)
{
    if (s_ap_started) {
        return ESP_OK;
    }

    wifi_mode_t mode = s_wifi_credential_count > 0 ? WIFI_MODE_APSTA : WIFI_MODE_AP;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(mode), TAG, "failed to enable setup AP mode");
    ESP_RETURN_ON_ERROR(configure_setup_ap(), TAG, "failed to configure setup AP");
    return start_provisioning_services();
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if (s_wifi_credential_count > 0) {
            esp_wifi_connect();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        if (s_sta_retries < CONFIG_ESP_ARC452A21_WIFI_MAX_RETRIES) {
            ++s_sta_retries;
            ESP_LOGW(TAG, "WiFi disconnected; reconnecting (%d/%d)",
                     s_sta_retries, CONFIG_ESP_ARC452A21_WIFI_MAX_RETRIES);
            esp_wifi_connect();
        } else {
            connect_next_station_or_start_portal();
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "Setup AP started");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        s_sta_connected = true;
        s_sta_retries = 0;
        ESP_LOGI(TAG, "WiFi connected, IP " IPSTR, IP2STR(&event->ip_info.ip));
        if (start_http_server() != ESP_OK) {
            ESP_LOGW(TAG, "HTTP server failed to start");
        }
        if (start_mqtt_client() != ESP_OK) {
            ESP_LOGW(TAG, "MQTT client failed to start");
        }
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs reinitialization: %s", esp_err_to_name(err));
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "failed to erase NVS");
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t iot_remote_start(const iot_remote_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is required");
    ESP_RETURN_ON_FALSE(config->command_handler != NULL, ESP_ERR_INVALID_ARG,
                        TAG, "command handler is required");

    s_config = *config;

    ESP_RETURN_ON_ERROR(init_nvs(), TAG, "failed to initialize NVS");
    load_mqtt_settings();
    load_device_hostname_setting();
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "failed to initialize netif");

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "failed to create default event loop");
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();
    s_ap_netif = esp_netif_create_default_wifi_ap();
    ESP_RETURN_ON_FALSE(s_sta_netif != NULL, ESP_ERR_NO_MEM, TAG, "failed to create station netif");
    ESP_RETURN_ON_ERROR(esp_netif_set_hostname(s_sta_netif, s_device_hostname),
                        TAG, "failed to set station hostname");
    ESP_LOGI(TAG, "WiFi hostname set to \"%s\"", s_device_hostname);

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init), TAG, "failed to initialize WiFi");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                   wifi_event_handler, NULL),
                        TAG, "failed to register WiFi handler");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                   wifi_event_handler, NULL),
                        TAG, "failed to register IP handler");

    load_station_credentials();

    if (s_wifi_credential_count > 0) {
        ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "failed to set WiFi mode");
        ESP_RETURN_ON_ERROR(apply_current_station_config(), TAG, "failed to set WiFi config");
        ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "failed to start WiFi");

        ESP_LOGI(TAG, "WiFi remote control connecting to SSID \"%s\"",
                 s_wifi_credentials[s_current_wifi_credential].ssid);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "failed to set AP mode");
    ESP_RETURN_ON_ERROR(configure_setup_ap(), TAG, "failed to configure setup AP");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "failed to start WiFi AP");
    return start_provisioning_services();
}

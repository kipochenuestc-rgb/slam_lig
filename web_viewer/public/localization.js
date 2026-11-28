const $ = id => document.getElementById(id);
let bag = '', duration = 0, rows = [], version = 0, inspection = 0;
let timer = null, current = null, lastTick = 0;
async function api(path, options) {
    const response = await fetch(path, options);
    if (!response.ok) throw Error((await response.json()).detail || '读取失败');
    return response.json();
}
function options(select, items) {
    select.replaceChildren(...items.map(item => new Option(item.label, item.id)));
}
function pause() {
    clearInterval(timer); timer = null; $('play').textContent = '播放';
}
function send() {
    $('map').contentWindow?.postMessage({type:'localization-position', xyz:current}, location.origin);
}
function showTime() {
    const t = Number($('time').value);
    $('stamp').textContent = t.toFixed(2) + ' / ' + duration.toFixed(2) + ' s';
    let lo = 0, hi = rows.length;
    while (lo < hi) {
        const mid = Math.floor((lo + hi)/2);
        if (rows[mid][0] <= t) lo = mid + 1;
        else hi = mid;
    }
    const sample = rows[lo - 1];
    current = sample?.[2] || null;
    $('pos').textContent = current ? current.map((v,i) => 'XYZ'[i] + ' ' + v.toFixed(2) + ' m').join('　') : '当前无有效定位';
    $('detail').textContent = sample ? sample[1] + ' · 最近采样 ' + sample[0].toFixed(2) + ' s · 距当前 ' + (t-sample[0]).toFixed(2) + ' s' : '';
    send();
}
function resetSamples() {
    pause(); version++; rows = [];
    $('time').disabled = $('play').disabled = true;
    showTime();
}
function baseMap() {
    const dataset = $('maps').value;
    $('map').src = dataset ? '/atlas/' + encodeURIComponent(dataset) + '/index.html?localization=1&view=' + ($('sat').checked?'satellite':'local') : 'about:blank';
}
async function positions() {
    resetSamples();
    const id = version;
    if (!bag || !$('maps').value || !$('topic').value) return;
    $('status').textContent = '正在读取所选 topic…';
    try {
        const data = await api('/api/localization/positions?' + new URLSearchParams({directory:bag,topic:$('topic').value,dataset:$('maps').value}));
        if (id !== version) return;
        rows = data.samples;
        if (rows.length && Number($('time').value) === 0) {
            const first = rows.find(sample => sample[2]);
            if (first) $('time').value = Math.min(duration, Math.ceil(first[0]*100)/100);
        }
        $('status').textContent = rows.length ? '地图原点 ' + data.origin.latitude + '°, ' + data.origin.longitude + '°' : 'topic 无采样';
        $('time').disabled = $('play').disabled = !rows.length;
        showTime();
    } catch(e) { if (id === version) $('status').textContent = e.message; }
}
async function inspect() {
    const id = ++inspection;
    resetSamples(); bag = ''; options($('topic'), []);
    const directory = $('bag').value.trim();
    $('status').textContent = '正在读取 bag…';
    try {
        const info = await api('/api/localization/bag?' + new URLSearchParams({directory}));
        if (id !== inspection) return;
        bag = directory; duration = info.duration;
        $('time').max = duration; $('time').value = 0;
        options($('topic'), info.topics.map(name => ({id:name,label:name})));
        $('status').textContent = info.topics.length ? '请选择 Autoware 地图' : 'bag 内没有 NavSatFix topic';
        await positions();
    } catch(e) { if (id === inspection) $('status').textContent = e.message; }
}
async function maps(selected) {
    const items = await api('/api/localization/maps');
    options($('maps'), items);
    if (selected) $('maps').value = selected;
    baseMap();
}
$('bagForm').onsubmit = e => { e.preventDefault(); inspect(); };
$('browse').onclick = async () => {
    const picker = parent.pywebview?.api;
    if (!picker) { $('bag').focus(); $('status').textContent='请粘贴 bag 文件夹的完整路径，再点击读取 bag'; return; }
    const path = await picker.choose_directory();
    if (path) { $('bag').value = path; inspect(); }
};
$('import').onclick = async () => {
    $('importForm').style.display='flex'; $('dir').focus();
    if (parent.pywebview?.api) {
        const path = await parent.pywebview.api.choose_directory();
        if (path) { $('dir').value=path; $('importForm').requestSubmit(); }
    }
};
$('importForm').onsubmit = async e => {
    e.preventDefault();
    try {
        const result = await api('/api/datasets/import',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({directory:$('dir').value.trim()})});
        await maps(result.id); $('importForm').style.display='none'; await positions();
    } catch(error) { $('status').textContent=error.message; }
};
$('maps').onchange = () => { baseMap(); positions(); };
$('topic').onchange = positions;
$('sat').onchange = baseMap;
$('time').oninput = () => { lastTick = performance.now(); showTime(); };
$('play').onclick = () => {
    if (timer) { pause(); return; }
    if (Number($('time').value) >= duration - .01) $('time').value = 0;
    lastTick = performance.now(); $('play').textContent='暂停';
    timer = setInterval(() => {
        const now=performance.now(), t=Number($('time').value)+(now-lastTick)/1000;
        lastTick=now; $('time').value=Math.min(t,duration); showTime();
        if (t >= duration) pause();
    },80);
};
window.addEventListener('message',e=>{
    if (e.origin===location.origin && e.source===$('map').contentWindow && e.data?.type==='localization-ready') send();
});
try {
    await maps();
    const active = await api('/api/mapping');
    if (active.bag && !$('bag').value) { $('bag').value = active.bag; await inspect(); }
} catch(e) { $('status').textContent = e.message; }

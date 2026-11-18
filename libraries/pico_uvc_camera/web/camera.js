// The snap command streams the JPEG back: "/rc/snap\0<size>", then the bytes.
// snap/<w>x<h> asks for a size; camera/sizes lists the ones the camera offers.
var cameraUrl = null;
var cameraAsked = 0;
var cameraSizesKnown = false;

// Live view: the next snap goes out as soon as the last picture is in, so only
// one is ever on its way and the rate follows the link. A failed one waits
// (0.5 s, doubling to 5 s) before trying again. The board always answers
// (a picture, or "no picture" after its own timeout) unless the socket goes,
// so a snap is only sent again when the one on its way rode an older socket
// (or after CAMERA_LIVE_LOST_MS, in case one goes missing anyway).
var CAMERA_LIVE_CHECK_MS = 2000;
var CAMERA_LIVE_LOST_MS = 30000;
var cameraLive = { on: false, timer: null, backoff: 0, frames: 0, bytes: 0, ms: 0, since: 0, last: "", ticker: null,
                   waiting: false, ws: null, sent: 0 };

function cameraSavedSize() {
    try { return localStorage.getItem("cameraSize") || "320x240"; } catch (e) { return "320x240"; }
}

function cameraSizeChosen() {
    try { localStorage.setItem("cameraSize", document.getElementById("cameraSize").value); } catch (e) {}
}

function cameraFillSizes(sizes) {
    var select = document.getElementById("cameraSize");
    var want = cameraSavedSize();
    select.innerHTML = "";
    sizes.forEach(function (size) {
        var option = document.createElement("option");
        option.value = size;
        option.textContent = size;
        select.appendChild(option);
    });
    select.value = (sizes.indexOf(want) >= 0) ? want : (sizes.indexOf("320x240") >= 0 ? "320x240" : sizes[0]);
}

function cameraAskSizes() {
    handler.send(new Blob(["/rc/camera/sizes"]), T("lbl.camStatus"), true, false);
}

onStreamedBlob("/rc/snap", function (blob) {
    // A failed snap answers with text instead: "no picture: <why>".
    if (typeof blob === "string") {
        if (cameraLive.on) {
            if (cameraLiveAnswered()) cameraLiveFailed(blob);
            return;
        }
        document.getElementById("cameraInfo").textContent = blob;
        hudToast("bad", blob);
        return;
    }
    if (cameraUrl) URL.revokeObjectURL(cameraUrl);
    cameraUrl = URL.createObjectURL(blob);
    var image = document.getElementById("cameraImage");
    if (cameraLive.on) {
        cameraLiveGot(blob.size);
        image.onload = null;
        image.src = cameraUrl;
        return;
    }
    image.onload = function () {
        var ms = cameraAsked ? (Date.now() - cameraAsked) : 0;
        document.getElementById("cameraInfo").textContent = image.naturalWidth + "x" + image.naturalHeight + ", " +
            fmtBytes(blob.size) + (ms ? ", " + ms + " ms" : "") + ", " + new Date().toLocaleTimeString();
    };
    image.src = cameraUrl;
});

// camera answers either its sizes ("sizes: 160x120 320x240 ...") or its status line.
onMessage("/rc/camera", function (text) {
    if (text.indexOf("sizes:") === 0) {
        var sizes = text.substring(6).trim().split(/\s+/).filter(function (s) { return s.length > 0; });
        if (sizes.length > 0) {
            cameraSizesKnown = true;
            cameraFillSizes(sizes);
        }
        return;
    }
    document.getElementById("cameraInfo").textContent = text;
});

// Once connected (the config arrives first): what sizes does the camera have?
onConfigLoaded(function () {
    document.getElementById("cameraSize").value = cameraSavedSize();
    cameraAskSizes();
});

function cameraSnap() {
    if (!cameraSizesKnown) cameraAskSizes();     // the camera may have come up since
    cameraAsked = Date.now();
    var size = document.getElementById("cameraSize").value;
    sendCommand("snap/" + size, T("lbl.snap"));
}

function cameraStatus() {
    sendCommand("camera", T("lbl.camStatus"));
}

function cameraLiveToggle() {
    if (cameraLive.on) {
        cameraLiveStop();
        return;
    }
    if (!cameraSizesKnown) cameraAskSizes();
    cameraLive.on = true;
    cameraLive.backoff = 0;
    cameraLive.frames = 0;
    cameraLive.bytes = 0;
    cameraLive.ms = 0;
    cameraLive.since = Date.now();
    cameraLive.last = "";
    cameraLive.ticker = setInterval(cameraLiveTick, 1000);
    cameraLiveLabel("cam.liveStop");
    cameraLiveNext();
}

function cameraLiveStop() {
    cameraLive.on = false;
    if (cameraLive.timer) clearTimeout(cameraLive.timer);
    if (cameraLive.ticker) clearInterval(cameraLive.ticker);
    cameraLive.timer = null;
    cameraLive.ticker = null;
    cameraLive.waiting = false;
    cameraLiveLabel("cam.live");
}

// data-t too, so a skin change keeps the right label.
function cameraLiveLabel(key) {
    var button = document.getElementById("cameraLive");
    button.setAttribute("data-t", key);
    button.value = T(key);
}

// The Camera tab is the one showing, and the browser tab is in front.
function cameraShown() {
    var tab = document.getElementById("Camera");
    return !document.hidden && tab && tab.offsetParent !== null;
}

function cameraLiveNext() {
    if (cameraLive.timer) clearTimeout(cameraLive.timer);
    cameraLive.timer = null;
    if (!cameraLive.on) return;
    if (!cameraShown()) {
        cameraLiveStop();
        return;
    }

    cameraAsked = Date.now();
    var size = document.getElementById("cameraSize").value;
    // quiet and not locking: no toast or busy page for every picture.
    if (!handler.send(new Blob(["/rc/snap/" + size]), T("lbl.snap"), true, false)) {
        cameraLiveStop();
        return;
    }
    cameraLive.waiting = true;
    cameraLive.ws = handler.ws;
    cameraLive.sent = Date.now();
    cameraLive.timer = setTimeout(cameraLiveCheck, CAMERA_LIVE_CHECK_MS);
}

// Still no answer: one that can never come (its socket is gone) is asked again.
function cameraLiveCheck() {
    cameraLive.timer = null;
    if (!cameraLive.on || !cameraLive.waiting) return;
    var open = handler.ws && (handler.ws.readyState === WebSocket.OPEN);
    if ((open && (handler.ws !== cameraLive.ws)) || ((Date.now() - cameraLive.sent) > CAMERA_LIVE_LOST_MS)) {
        cameraLive.waiting = false;
        cameraLiveFailed(T("cam.liveLost"));
        return;
    }
    cameraLive.timer = setTimeout(cameraLiveCheck, CAMERA_LIVE_CHECK_MS);     // not reconnected yet, or still coming
}

// An answer counts only while one is expected: a late one after a resend is shown, not followed up.
function cameraLiveAnswered() {
    if (!cameraLive.waiting) return false;
    cameraLive.waiting = false;
    return true;
}

function cameraLiveGot(size) {
    if (!cameraLiveAnswered()) return;
    cameraLive.frames += 1;
    cameraLive.bytes += size;
    cameraLive.ms += Date.now() - cameraAsked;
    cameraLive.backoff = 0;
    cameraLive.last = "";
    cameraLiveNext();
}

function cameraLiveFailed(why) {
    if (!cameraLive.on) return;
    cameraLive.waiting = false;
    if (cameraLive.timer) clearTimeout(cameraLive.timer);
    // Toast once per run of failures, not per retry.
    if (cameraLive.backoff === 0) hudToast("bad", why);
    cameraLive.last = why;
    cameraLive.backoff = cameraLive.backoff ? Math.min(cameraLive.backoff * 2, 5000) : 500;
    cameraLive.timer = setTimeout(cameraLiveNext, cameraLive.backoff);
}

// Once a second: the rate and sizes since the last tick.
function cameraLiveTick() {
    if (!cameraLive.on) return;
    if (!cameraShown()) {
        cameraLiveStop();
        return;
    }
    var now = Date.now();
    var secs = Math.max((now - cameraLive.since) / 1000, 0.001);
    var image = document.getElementById("cameraImage");
    var text = T("cam.liveOn") + " " + (cameraLive.frames / secs).toFixed(1) + " fps";
    if (cameraLive.frames > 0) {
        text += ", " + image.naturalWidth + "x" + image.naturalHeight + ", " + fmtBytes(Math.round(cameraLive.bytes / cameraLive.frames)) +
            ", " + Math.round(cameraLive.ms / cameraLive.frames) + " ms";
    }
    if (cameraLive.last) text += ", " + cameraLive.last;
    document.getElementById("cameraInfo").textContent = text;
    cameraLive.frames = 0;
    cameraLive.bytes = 0;
    cameraLive.ms = 0;
    cameraLive.since = now;
}

document.addEventListener("visibilitychange", function () {
    if (document.hidden && cameraLive.on) cameraLiveStop();
});

var cameraRotation = cameraSavedRotation();

function cameraSavedRotation() {
    try { return ((parseInt(localStorage.getItem("cameraRotation"), 10) || 0) % 360 + 360) % 360; } catch (e) { return 0; }
}

function cameraRotate(step) {
    cameraRotation = ((cameraRotation + step) % 360 + 360) % 360;
    try { localStorage.setItem("cameraRotation", String(cameraRotation)); } catch (e) {}
    cameraApplyRotation();
}

function cameraApplyRotation() {
    var image = document.getElementById("cameraImage");
    if (!image) return;
    image.style.transform = cameraRotation ? "rotate(" + cameraRotation + "deg)" : "";
    image.style.maxWidth = "";
    image.style.marginTop = "";
    image.style.marginBottom = "";
    if (cameraRotation % 180 === 0 || !image.naturalWidth || !image.naturalHeight) return;
    var room = image.parentNode.clientWidth;
    image.style.maxWidth = Math.min(room, room * image.naturalWidth / image.naturalHeight) + "px";
    var shift = Math.max((image.offsetWidth - image.offsetHeight) / 2, 0) + "px";
    image.style.marginTop = shift;
    image.style.marginBottom = shift;
}

document.addEventListener("load", function (event) {
    if (event.target && event.target.id === "cameraImage") cameraApplyRotation();
}, true);
window.addEventListener("resize", cameraApplyRotation);

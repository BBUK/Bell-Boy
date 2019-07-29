// Copyright (c) 2017,2018,2019 Peter Budd. All rights reserved
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//     * The above copyright notice and this permission notice shall be included in all copies or substantial
//       portions of the Software.
//     * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//       BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//       IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//       WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//       SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. THE AUTHORS AND COPYRIGHT HOLDERS, HOWEVER,
//       ACCEPT LIABILITY FOR DEATH OR PERSONAL INJURY CAUSED BY NEGLIGENCE AND FOR ALL MATTERS LIABILITY
//       FOR WHICH MAY NOT BE LAWFULLY LIMITED OR EXCLUDED UNDER ENGLISH LAW

// This script forms part of the Bell-Boy project to measure the force applied by a bell ringer to a tower
// bell rope.  The Bell-Boy uses rotational acceleration of the bell as a proxy for force applied.  The
// hardware is currently a Pi Zero running Arch Linux.

// I pulled snippets of code from loads of places so if any of you recognise anything you wrote - thanks!

// The script below is the javascript for the front end.  It makes extensive use of HTML5 canvases
// so HTML5 is required.  Websockets are also used to pull data from the Bell-Boy device.


var BGCOLOUR="#686888";

var nonLive=false;
var wsHost="10.0.0.1"

var sampleInterval = 0.008;  // seconds for each sample (default 125 times/sec).  Updated by SAMP: command
var collectInterval = 0.1; // update display in ms - here 10 times/sec
var batteryLevel = 100;
var sleepTime = 0.5;

var canvasBD = document.getElementById("canvasBD");
var ctxBD = canvasBD.getContext("2d");

var canvasBDt= document.getElementById("canvasBDt");
var ctxBDt = canvasBDt.getContext("2d");

var posBS2 = 0; // position on canvas of BS2
var posHS1 = 0;
var posHS2 = 0;
var posBS1 = 0;
var posCB = 0;  // position on canvas of bell (and timers)
var CBwidth = 64; // width of bell on canvas
var BDwidth = 0; // width of individual stroke sections on canvas

var strokeTimer=0; // used for time displays under animated bell

var canvasAT=document.getElementById("canvasAT");
var ctxAT=canvasAT.getContext("2d");
var canvasATt=document.getElementById("canvasATt");
var ctxATt=canvasATt.getContext("2d");

var currentATmargin = 0;
var currentATpixels = 1;
var ATbottomMargin=45; // pixels at bottom of AT canvas
var ATtopMargin = 30; // pixels at top of AT canvas
var ATstepWidth = 30;

var wsOpened = false;

//var lastWinWidth =  window.innerWidth;

var scaleValue = 0.0;

var ROIRanges = [[-20,90],[-20,70],[-10,60],[-10,50],[-10,40],[-10,30]];
var currentROI=3;
var ROIU = ROIRanges[currentROI][0];
var ROIL = ROIRanges[currentROI][1];

var playbackRanges = [200, 150, 100, 80, 50, 30, 10]; // available playback speeds (percent)

var currentPlaybackSpeed = 2;
var targets = [ "none", "-10", "-7", "-5", "-2" , "0", "2", "5", "7" , "10" ]; // these are target balance angles
var scales = [ 300, 200, 100, 80, 50, 30 ];

var currentPlaybackPosition = 1;
var playintervalID = null;
var statusintervalID = null;
var liveintervalID = null;

var changeInterval = null;
var CPM = 31.0;
var changeStep = null;
var openHandstroke = 1.0;

var TchangeInterval = null;
var TopenHandstroke = null;
var TchangeStep = null;

var gravityValue = 0.0;
var ropeValueB = 0.0;
var ropeValueH = 0.0;

var currentStatus = 0;
var DOWNLOADINGFILE=1;
var RECORDINGSESSION=2;
var SESSIONLOADED=4;
var TEMPLATEDISPLAYED=8;
var PLAYBACK=16;
var LASTHS1=32;
var LASTHS2=64;
var LASTBS1=128;
var LASTBS2=256;
var HELPDISPLAYED=512;
var PAUSED=1024;
// var GRIDLINESDISPLAYED=2048;
//var LIVEVIEW=4096;
var ABORTFLAG = 8192;
var SKIPMAIN = 16384;
var SKIPTEMPLATE = 32768;
var WOBBLINGATSTAND = 65536;

var targetAngleHand=null;
var targetAngleBack=null;

var currentSwingDisplayed=null;
var TcurrentSwingDisplayed=null;

var sample = [];
var ringTimes = [];
var switchPoints = [];
var template = [];
var TringTimes = [];
var TswitchPoints = [];

var swingStarts = [];
var TswingStarts = [];

var averagePullStrength=[];
var TaveragePullStrength=[];

var fileList=[];

ws = new WebSocket("ws://" + wsHost);

////////////////////////////////////////////////////////////////
//                SETUP SELECT BOX FUNCTIONS                  //
////////////////////////////////////////////////////////////////

document.getElementById("targetSelectHand").options.length = 0;
document.getElementById("targetSelectBack").options.length = 0;
for (var i=0; i < targets.length; i++) {
    var option = document.createElement("option");
    option.text=targets[i];
    document.getElementById("targetSelectHand").add(option);
    option = document.createElement("option");
    option.text=targets[i];
    document.getElementById("targetSelectBack").add(option);
}

document.getElementById("zoomSelect").options.length = 0;
for (var i=0; i < ROIRanges.length; i++) {
    var option = document.createElement("option");
    option.text=ROIRanges[i][0].toString() + " to " + ROIRanges[i][1].toString();
    document.getElementById("zoomSelect").add(option);
}
document.getElementById("zoomSelect").selectedIndex = currentROI.toString();

document.getElementById("bellsSelect").options.length = 0;
for (var i=3; i<13; i++) {
    var option = document.createElement("option");
    option.text=i.toString();
    document.getElementById("bellsSelect").add(option);
}
document.getElementById("bellsSelect").selectedIndex = 3; // default number of bells = 6 

var option = document.createElement("option");
option.text="Auto";
document.getElementById("chimeSelect").add(option);
for (var i=250; i < 500; i += 20) {
    var option = document.createElement("option");
    option.text=i.toString();
    document.getElementById("chimeSelect").add(option);
}
document.getElementById("chimeSelect").selectedIndex = 0;

document.getElementById("speedSelect").options.length = 0;
for (var i=0; i < playbackRanges.length; i++) {
    var option = document.createElement("option");
    option.text=playbackRanges[i].toString() + "%"
    document.getElementById("speedSelect").add(option);
}
document.getElementById("speedSelect").selectedIndex = currentPlaybackSpeed;

document.getElementById("scaleSelect").options.length = 0;
for (var i=0; i < scales.length; i++) {
    var option = document.createElement("option");
    option.text=scales[i].toString()
    document.getElementById("scaleSelect").add(option);
}
document.getElementById("scaleSelect").selectedIndex = 2;  // start at 100
scaleValue=parseFloat(scales[document.getElementById("scaleSelect").selectedIndex])

////////////////////////////////////////////////////////////////
//                 WEBSOCKET HANDLER FUNCTIONS                //
////////////////////////////////////////////////////////////////

ws.onopen = function(){
    wsOpened=true;
    setStatus("Link open");
    fileList=[];
    document.getElementById("openSelect").options.length = 0;
    ws.send("FILE:");
    var d = new Date();

    if (!nonLive){
        ws.send("DATE:" + parseInt(d.getTime()/1000));
        ws.send("SAMP:");
        calculateTimings();
    }
};

ws.onmessage = function (event) {
    var dataBack = event.data.split("\n");
    for (var i=0; i<dataBack.length; i++) {
        if (dataBack[i].length > 3) parseResult(dataBack[i]);
    }
};

function parseResult(dataBack) {
    if (dataBack.slice(0,5) == "FILE:"){
        fileList[fileList.length] = dataBack.slice(5); 
        return;
    }

    if (dataBack.slice(0,5) == "EFIL:"){
        fileList.sort();
        for(var i = 0; i<fileList.length; ++i){
            var option = document.createElement("option");
            option.text=fileList[i];
            document.getElementById("openSelect").add(option);
        }
        fileList=[];
        return;
    }

    if (dataBack.slice(0,5) == "DATA:"){
        var sampArray = dataBack.split("DATA:");
        for (var i = 0; i < sampArray.length; i++) {
            if (sampArray[i].length < 20) continue; // have more checks for good data
            var entries = sampArray[i].split(",");
            sample[sample.length] = [parseFloat(entries[0].slice(2)), parseFloat(entries[1].slice(2)), parseFloat(entries[2].slice(2)), parseFloat(entries[3].slice(2)), parseFloat(entries[4].slice(2))];
            var data = parseFloat(entries[3].slice(2));
            if(data != 0 || sample.length <= 8){
                var value = parseFloat(entries[4].slice(2));
                if(data == 1 || data == 2) ringTimes[ringTimes.length] = value;
                if(data == 7) switchPoints[switchPoints.length] = value;
                if(sample.length == 1) gravityValue = value;
                if(sample.length == 2)  {
                    if (value < 3) value = 3;
                    if (value > 12) value = 12;
                    document.getElementById("bellsSelect").selectedIndex = Math.round(value - 3);
                    calculateTimings();
                }
                if(sample.length == 3) {
                    if (value < 20) value = 20;
                    if (value > 50) value = 50;
                    CPM = Math.round(value);
                    calculateTimings()
                }
                if(sample.length == 4) {
                    openHandstroke = Math.round(value*10)/10;
                    if(openHandstroke < 0.0) openHandstroke = 0.0;
                    if(openHandstroke > 2.2) openHandstroke = 2.2;
                    calculateTimings();
                }
                if(sample.length == 5){
                    if(value < 250){
                        document.getElementById("chimeSelect").selectedIndex = 0; // auto detection
                    } else {
                        document.getElementById("chimeSelect").selectedIndex = Math.round(1+((value-250)/20));
                    }
                }
                //skip tare value
                if(sample.length == 7) ropeValueB = value;
                if(sample.length == 8) ropeValueH = value;
            }
        }
        setStatus("Loaded: " + sample.length.toString());
        return;
    }
    
    if (dataBack.slice(0,5) == "LIVE:"){
        if (!(currentStatus & RECORDINGSESSION)) return;  // server may push data after stopping
        var sampArray = dataBack.split("LIVE:");
        for (var i = 0; i < sampArray.length; i++) {
            if (sampArray[i].length < 20) continue; // have more checks for good data
            var entries = sampArray[i].split(",");
            sample[sample.length] = [parseFloat(entries[0].slice(2)), parseFloat(entries[1].slice(2)), parseFloat(entries[2].slice(2)), parseFloat(entries[3].slice(2)), parseFloat(entries[4].slice(2))];
            var data = parseFloat(entries[3].slice(2));
            if(data != 0 || sample.length <= 8){
                var value = parseFloat(entries[4].slice(2));
                if(data == 1 || data == 2) ringTimes[ringTimes.length] = value; // need to check if this is zero length
                if(data == 7) switchPoints[switchPoints.length] = value;
                if(sample.length == 1) gravityValue = value;
                if(sample.length == 2)  {
                    if (value < 3) value = 3;
                    if (value > 12) value = 12;
                    document.getElementById("bellsSelect").selectedIndex = Math.round(value - 3);
                    calculateTimings();
                }
                if(sample.length == 3) {
                    if (value < 20) value = 20;
                    if (value > 50) value = 50;
                    CPM = Math.round(value);
                    calculateTimings()
                }
                if(sample.length == 4) {
                    openHandstroke = Math.round(value*10)/10;
                    if(openHandstroke < 0.0) openHandstroke = 0.0;
                    if(openHandstroke > 2.2) openHandstroke = 2.2;
                    calculateTimings();
                }
                if(sample.length == 5){
                    if(value < 250){
                        document.getElementById("chimeSelect").selectedIndex = 0; // auto detection
                    } else {
                        document.getElementById("chimeSelect").selectedIndex = Math.round(1+((value-250)/20));
                    }
                }
                //skip tare value
                if(sample.length == 7) ropeValueB = value;
                if(sample.length == 8) ropeValueH = value;
            }
        }
        return;
    }

    if ((dataBack.slice(0,5) == "LFIN:") || (dataBack.slice(0,5) == "STPD:")) {
        if ((currentStatus & DOWNLOADINGFILE) !=0){
            currentStatus &= ~DOWNLOADINGFILE;
        } else if (currentStatus & RECORDINGSESSION) {
            currentStatus &= ~RECORDINGSESSION;
        }
        if ((currentStatus & ABORTFLAG) !=0) {
            currentStatus &= ~ABORTFLAG;
            sample = [];
            swingStarts=[];
            currentPlaybackPosition = 1;
            currentSwingDisplayed = null;
            clearStave();
            if(currentStatus & TEMPLATEDISPLAYED) drawStrokeT();
            return;
        }
        currentStatus |= SESSIONLOADED;
        currentStatus |= SKIPMAIN;
        updateIcons();
        swingStarts=[];
        swingStarts[0] = 3;
        currentPlaybackPosition = 1;
        currentSwingDisplayed = null;
        if((currentStatus & TEMPLATEDISPLAYED) && (dataBack.slice(0,5) == "LFIN:")) { clearStave(); drawStrokeT(); }
        if(sample.length < 100){
            setStatus("Insufficient data found!");
            return;
        }
        var j=0;
        var arrayLength = sample.length;
        while (j < arrayLength && sample[j][0] < 90.0) j++;  // move forward through sample to next swing
        for (; j < arrayLength; j++){
            if (sample[j][0] < 90.0 && sample[j][1] < 0.0) {
                swingStarts[swingStarts.length] = j;
                while (j < arrayLength && sample[j][0] < 90.0) j++;  // move forward through sample to next swing
                while (j < arrayLength && sample[j][0] >= 90.0) j++;
                if (j == arrayLength ) break;
            }
        }
        if (switchPoints.length == 0 || sample[switchPoints[0]][0] > 180) switchPoints.unshift(3); // a "just in case" should start at stand not be detected
        setStatus(swingStarts.length.toString() + " strokes found.");
        calculatePullStrengths();
        document.getElementById("openSelect").options.length = 0;
        ws.send("FILE:");
        return;
    }
    if (dataBack.slice(0,5) == "STRT:"){
        setStatus("Recording started");
        return;
    }
    if (dataBack.slice(0,5) == "EIMU:"){
        setStatus("IMU not active.  Is one connected to device? Aborting");
        if (currentStatus & RECORDINGSESSION) {
            document.getElementById("recordIcon").onclick()
            ws.send("STOP:");
        }
        currentStatus |= ABORTFLAG;
        return;
    }
    if (dataBack.slice(0,5) == "SAMP:"){
        sampleInterval = parseFloat(dataBack.slice(5));
        return;
    }
	
    if (dataBack.slice(0,5) == "BATT:"){
        batteryLevel = parseInt(dataBack.slice(5));
		document.getElementById("batteryIcon").title = "Battery: " + batteryLevel + "%";
		if(batteryLevel <= 10){
			document.getElementById("batteryIcon").src = "battery0.png"
		} else if(batteryLevel <= 40){
			document.getElementById("batteryIcon").src = "battery50.png"
		}
        return;
    }

    if (dataBack.slice(0,5) == "EWAI:"){
        setStatus("Now calibrating. Please keep bell down and at rest...");
        return;
    }

    if (dataBack.slice(0,5) == "EPUL:"){
        setStatus("Now calibrating. Please raise bell...");
        return;
    }

    if (dataBack.slice(0,5) == "ESET:"){
        setStatus("Now calibrating. Please set bell normally...");
        return;
    }

    if (dataBack.slice(0,5) == "ESWI:"){
        if(parseInt(dataBack.slice(5)) == 5) {
            setStatus("Now calibrating. Please swing bell at least " + parseInt(dataBack.slice(5)) + " times...");
        } else if (parseInt(dataBack.slice(5)) != 1){
            setStatus("Now calibrating. Please swing bell at least " + parseInt(dataBack.slice(5)) + " more times...");
        } else {
            setStatus("Now calibrating. Please swing bell at least " + parseInt(dataBack.slice(5)) + " more time...");
        }
        return;
    }

    if (dataBack.slice(0,5) == "EDEF:"){
        setStatus("Error loading calibration.  Default values used.");
        return;
    }

    if (dataBack.slice(0,5) == "EYEC:"){
        var entries = dataBack.split(",");

//        var roll  = 360-parseFloat(entries[0].slice(5));
//        var pitch = 360-parseFloat(entries[1]);
//        var yaw   = 0.0; //360-parseFloat(entries[2]);
//        document.querySelector("section").style.transform= "rotateX(" + roll + "deg) rotateZ(-" + pitch + "deg) rotateY(" + yaw + "deg)";
//        return;
        // https://www.allaboutcircuits.com/projects/bosch-absolute-orientation-sensor-bno055/
        var x = -parseFloat(entries[4]);
        var y = parseFloat(entries[6]);
        var z = parseFloat(entries[5]);
        var w = parseFloat(entries[3]);
        var x2 = x + x;
        var y2 = y + y;
        var z2 = z + z;
        var xx = x * x2;
        var xy = x * y2;
        var xz = x * z2;
        var yy = y * y2;
        var yz = y * z2;
        var zz = z * z2;
        var wx = w * x2;
        var wy = w * y2;
        var wz = w * z2;
        document.querySelector("section").style.transform= "matrix3D(" + (1 - (yy + zz)) + "," + (xy + wz) + "," + (xz - wy) + "," + 0 + "," + (xy - wz) + "," + (1 - (xx + zz)) + "," + (yz + wx) + "," + 0 + "," + (xz + wy) + "," + (yz - wx) + "," + (1-(xx + yy)) + "," + 0 + "," + 0 + "," + 0 + "," + 0 + "," + 1 +")";
//        document.querySelector("section").style.transform= "matrix3D(" + (1 - (yy + zz)) + "," + (xy - wz) + "," + (xz + wy) + "," + 0 + "," + (xy + wz) + "," + (1 - (xx + zz)) + "," + (yz - wx) + "," + 0 + "," + (xz - wy) + "," + (yz + wx) + "," + (1-(xx + yy)) + "," + 0 + "," + 0 + "," + 0 + "," + 0 + "," + 1 +")";
        return;
    }

    if (dataBack.slice(0,5) == "ESTD:"){
        setStatus("Bell not at stand.  Aborting");
        if ((currentStatus & RECORDINGSESSION) != 0) {
            document.getElementById("recordIcon").onclick()
        }
        ws.send("STOP:");
        currentStatus |= ABORTFLAG;
        return;
    }
    if (dataBack.slice(0,5) == "EMOV:"){
        setStatus("Bell moving.  Aborting");
        if ((currentStatus & RECORDINGSESSION) != 0) {
            document.getElementById("recordIcon").onclick()
        }
        ws.send("STOP:");
        currentStatus |= ABORTFLAG;
        return;
    }
    if (dataBack.slice(0,5) == "EOVF:"){
        setStatus("Data overflow!");
        return;
    }
    setStatus(dataBack);
};

ws.onclose = function(event){
    wsOpened=false;
    setStatus("Error. Please close any other Bell-Boy windows and press refresh.");
    if ((currentStatus & RECORDINGSESSION) != 0 ) {
        currentStatus &= ~RECORDINGSESSION;
        updateIcons();
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        clearBell();
        currentPlaybackPosition=1;
    }
};

////////////////////////////////////////////////////////////////
//                       DATA FUNCTIONS                       //
////////////////////////////////////////////////////////////////

function playbackSample(){
//         playintervalID=setInterval(playbackSample,sampleInterval*(100/playbackRanges[currentPlaybackSpeed][1]));
// sampleInterval*collectChunk*(100/playbackRanges[currentPlaybackSpeed][1])
    var normalChunk = collectInterval/sampleInterval;
    var iterations = Math.round((playbackRanges[currentPlaybackSpeed]/100.0) * normalChunk); // do this so that slower speeds are smoother
    if (currentPlaybackPosition + iterations > sample.length-1){
        iterations = sample.length-currentPlaybackPosition-1;
    }
    drawSamples(currentPlaybackPosition,iterations);
    currentPlaybackPosition += iterations;
    if (currentPlaybackPosition >= sample.length-1){ // reached end of sample, so stop
        currentStatus &= ~PLAYBACK
        updateIcons();
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        currentPlaybackPosition =1;  // reset to 1 so that dataEntryOld works
    }
}

function calculatePullStrengths(){
    pullStrengths=[];
    var totalPull = 0.0;
    if(switchPoints.length == 0) return;
    var i = null, j=null, k=null,m = null;
    for (j=0; j<switchPoints.length; ++j){
        totalPull=0.0;
        i=sample[switchPoints[j]][0]; // angle at direction change
//        for (k=switchPoints[j]; k>0 && Math.abs(sample[k][0] - i) < 3 ;k--); // find point 5 degrees back from direction change
//        totalPull = sample[k][1]*sample[k][1];
//        for (m=1;(k+m)<sample.length && Math.abs(sample[k+m][0] - i) < 3; m++); // add up accns to point 5 degrees forward from direction change
//        totalPull += sample[m][1]*sample[m][1];

        for (k=switchPoints[j]; k>0 && Math.abs(sample[k][0] - i) < 5 ;k--); // find point 5 degrees back from direction change
        for (m=1;(k+m)<sample.length && Math.abs(sample[k+m][0] - i) < 5; m++){ // add up accns to point 5 degrees forward from direction change
            pull = Math.abs(sample[k+m][2]);
            if(pull < 25) continue;  // ignore stuff below noise floor
            totalPull += pull;
        }
        pullStrengths[pullStrengths.length] = totalPull/m;
//        console.log((totalPull/m).toString() + " ");
    }
}

function showLive(){
    var iterations = (sample.length-1) - currentPlaybackPosition;
    if (iterations < 5) return; // draw not fewer than 5 samples at a time
    drawSamples(currentPlaybackPosition,iterations);
    currentPlaybackPosition += iterations;
    setStatus("Loaded: " + currentPlaybackPosition.toString());
}

function drawTimer(position, templated){
    var timeElapsed = ((position-strokeTimer)*sampleInterval);
    if (timeElapsed < 0.2) return; // assume a glitch
    ctxBD.font = "12px sans serif";
    ctxBD.fillStyle = "white";
    if ((currentStatus & LASTBS2) != 0 ) {  // i.e. timing backstroke
        ctxBD.clearRect(posCB+1, ctxBD.canvas.height-20, CBwidth-2, 10);
        if (timeElapsed > 0.5) {
            ctxBD.textAlign = "center";
            ctxBD.fillText("B " +timeElapsed.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-10);
        }
    } else {
        ctxBD.clearRect(posCB+1, ctxBD.canvas.height-30, CBwidth-2, 10);
        if (timeElapsed > 0.5) {
            ctxBD.textAlign = "center";
            ctxBD.fillText("H " + timeElapsed.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-20);
        }
    }
    strokeTimer=position;
}

function drawStroke(){
    if (currentSwingDisplayed == null) return;
    var startpoint = swingStarts[currentSwingDisplayed];
    var endpoint = 0;
    if(currentSwingDisplayed == swingStarts.length-1) {
        endpoint = sample.length - 3
    } else {
        endpoint = swingStarts[currentSwingDisplayed+1];
    }
    drawSamples(startpoint,endpoint-startpoint);
    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);

    ctxBD.clearRect(posCB+1, ctxBD.canvas.height-30, CBwidth-2, 20);
    ctxBD.textAlign = "center";
    ctxBD.font = "12px sans serif";
    ctxBD.fillStyle = "white";
    var handstrokelength = 0.0;
    var backstrokelength = 0.0;
    if(ringTimes.length > (currentSwingDisplayed * 2)) handstrokelength = ringTimes[(currentSwingDisplayed * 2)];
    if(ringTimes.length > (currentSwingDisplayed * 2)+1) backstrokelength = ringTimes[(currentSwingDisplayed * 2)+1];
    if(handstrokelength != 0.0){
        ctxBD.fillText("H " + handstrokelength.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-20);
    }
    if(backstrokelength != 0.0){
        ctxBD.fillText("B " + backstrokelength.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-10);
    }
    clearStave();
    ctxAT.font = "14px sans serif";
    ctxAT.fillStyle = "white";
    ctxAT.textAlign = "start";
    ctxAT.textBaseline="bottom";
    
    var totalHeight = ctxAT.canvas.height-ATbottomMargin-ATtopMargin;
    var range = totalHeight/4.0; //(0.833*totalHeight)/5.0;  //range of +- 2 change "steps"
    var position = 0.0;
    var overflow = 0;
    var colour = "white";
    var time = 0;
    var cross = 0;
    var currentDrawRing = 0;
    var overflow = 0;
    var time = 0;
    var height = 0;
    var rise = 0;
    var i = 0;
    var offset = 0;
    i = Math.floor(-(ctxATt.canvas.width/2)/ATstepWidth)-1;

    for(;;i++){
        cross = 0;
        overflow = 0;
        currentDrawRing = (currentSwingDisplayed*2)+i;
//        if((Math.floor(i/2)+currentSwingDisplayed)+1 > swingStarts.length -1) break;
//        if((Math.floor(i/2)+currentSwingDisplayed)+1 < 0) continue;
        if(currentDrawRing < 0) continue;
        if(currentDrawRing > ringTimes.length -1) break;
        if(currentDrawRing > switchPoints.length -1) break;
        if(currentDrawRing > pullStrengths.length -1) break;
        offset = (i*ATstepWidth)+ctxATt.canvas.width/2;
        if(offset > ctxATt.canvas.width) break;
        if(offset < -ATstepWidth) continue;
        time = ringTimes[currentDrawRing];
        height = 10 + 25 * (pullStrengths[currentDrawRing]/scaleValue);
        if(height > 30) height = 30;
        
        if(!(i%2)){
            position = -range*(time - (changeInterval + (openHandstroke*changeStep)))/changeStep;
            rise = sample[switchPoints[currentDrawRing]][0] * (height/10.0); // show a rise of +-10 degrees (consider making this +- 20)
        } else {
            position = -range*(time - changeInterval)/changeStep;
            rise = (360-sample[switchPoints[currentDrawRing]][0]) * (height/10.0); // show a rise of +-10 degrees (consider making this += 20)
        }
        if(rise >= height) rise = height;
        if(rise <= -height) rise = -height;

        if(position < -totalHeight/2.0){
            overflow = 1;
            position = -totalHeight/2.0;
            height = 10;
        }
        if(position > totalHeight/2.0){
            overflow = 1;
            position = totalHeight/2.0
            height = 10;
        }
        position += (ATtopMargin + totalHeight/2.0);
        colour = "white";
        if(overflow) {colour = "rgba(240,128,128,1)"; cross = 1; }
        drawBar(offset,ctxAT,!(i%2),position,height,rise,cross,colour);
        if(!(i%2)){
            ctxAT.fillText((Math.floor(i/2)+currentSwingDisplayed+1).toFixed(0), offset-20, ctxAT.canvas.height);
        }
    }
    ctxAT.beginPath();
}

function drawStrokeT(){
    if (TcurrentSwingDisplayed == null) return;
    if ((currentStatus & TEMPLATEDISPLAYED) == 0) return;
    var startpoint = TswingStarts[TcurrentSwingDisplayed];
    var endpoint = 0;
    if(TcurrentSwingDisplayed == TswingStarts.length-1) {
        endpoint = template.length - 3
    } else {
        endpoint = TswingStarts[TcurrentSwingDisplayed+1];
    }

    drawSamplesOnTemplate(startpoint,endpoint-startpoint);

    ctxBD.clearRect(posCB+1, ctxBD.canvas.height-60, CBwidth-2, 20);
    ctxBD.textAlign = "center";
    ctxBD.font = "12px sans serif";
    ctxBD.fillStyle = "rgba(240,240,0,1)";
    var handstrokelength = 0.0;
    var backstrokelength = 0.0;
    if(TringTimes.length > (TcurrentSwingDisplayed * 2)) handstrokelength = TringTimes[(TcurrentSwingDisplayed * 2)];
    if(TringTimes.length > (TcurrentSwingDisplayed * 2)+1) backstrokelength = TringTimes[(TcurrentSwingDisplayed * 2)+1];
    if(handstrokelength != 0.0){
        ctxBD.fillText("H " + handstrokelength.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-50);
    }
    if(backstrokelength != 0.0){
        ctxBD.fillText("B " + backstrokelength.toFixed(2)+"s", 2*BDwidth + 32, ctxBD.canvas.height-40);
    }
    ctxAT.font = "14px sans serif";
    ctxAT.textAlign = "start";
    ctxAT.textBaseline="bottom";
    ctxAT.fillStyle = "rgba(240,240,0,0.6)";
    ctxAT.strokeStyle = "rgba(240,240,0,0.6)";
    ctxAT.beginPath();
    var totalHeight = ctxAT.canvas.height-ATbottomMargin-ATtopMargin;
    var range = totalHeight/4.0; //(0.833*totalHeight)/5.0;  //range of +- 2 change "steps"
    var position = 0.0;
    var overflow = 0;
    var time = 0;
    var cross = 0;
    var currentDrawRing = 0;
    var overflow = 0;
    var time = 0;
    var height = 0;
    var rise = 0;
    var i = 0;
    var offset = 0;
    i = Math.floor(-(ctxATt.canvas.width/2)/ATstepWidth)-1;

    for(;;i++){
        cross = 0;
        overflow = 0;
        currentDrawRing = (TcurrentSwingDisplayed*2)+i;
//        if((Math.floor(i/2)+currentSwingDisplayed)+1 > swingStarts.length -1) break;
//        if((Math.floor(i/2)+currentSwingDisplayed)+1 < 0) continue;
        if(currentDrawRing < 0) continue;
        if(currentDrawRing > TringTimes.length -1) break;
        if(currentDrawRing > TswitchPoints.length -1) break;
        if(currentDrawRing > TpullStrengths.length -1) break;
        offset = (i*ATstepWidth)+ctxATt.canvas.width/2;
        if(offset > ctxATt.canvas.width) break;
        if(offset < -ATstepWidth) continue;
        time = TringTimes[currentDrawRing];
        height = 10 + 25 * (TpullStrengths[currentDrawRing]/scaleValue);
        if(height > 30) height = 30;

        if(!(i%2)){
            position = -range*(time - (TchangeInterval + (TopenHandstroke*TchangeStep)))/TchangeStep;
            rise = template[TswitchPoints[currentDrawRing]][0] * (height/10.0); // show a rise of +-10 degrees (consider making this += 20)
        } else {
            position = -range*(time - TchangeInterval)/TchangeStep;
            rise = (360-template[TswitchPoints[currentDrawRing]][0]) * (height/10.0); // show a rise of +-10 degrees (consider making this += 20)
        }
        if(rise >= height) rise = height;
        if(rise <= -height) rise = -height;

        if(position < -totalHeight/2.0){
            overflow = 1;
            position = -totalHeight/2.0;
        }
        if(position > totalHeight/2.0){
            overflow = 1;
            position = totalHeight/2.0
        }
        position += (ATtopMargin + totalHeight/2.0);
        if(overflow) {cross = 1; height = 3;}
        if(!(i%2)){
            if(cross){
                ctxAT.moveTo(offset-16, position-height);
                ctxAT.lineTo(offset-10, position+height);
                ctxAT.moveTo(offset-10, position-height);
                ctxAT.lineTo(offset-16, position+height);
            } else {
                ctxAT.fillRect(offset-16, position+rise, 6, height-rise);

            }
            ctxAT.rect(offset-16, position-height, 6, 2 * height);
            ctxAT.fillText((Math.floor(i/2)+TcurrentSwingDisplayed+1).toFixed(0), offset+5, ctxAT.canvas.height);
        } else {
            if(cross){
                ctxAT.moveTo(offset-18, position-height);
                ctxAT.lineTo(offset-12, position+height);
                ctxAT.moveTo(offset-12, position-height);
                ctxAT.lineTo(offset-18, position+height);
            } else {
                ctxAT.fillRect(offset-18, position+rise, 6, height-rise);

            }
            ctxAT.rect(offset-18, position-height, 6, 2 * height);
        }
        ctxAT.stroke();
    }
    ctxAT.beginPath();
}

function calculateTimings(){
    var numberOfBells = document.getElementById("bellsSelect").selectedIndex + 3;
    changeInterval = 60.0/CPM;
    changeStep = changeInterval/numberOfBells;
    var pealTime = 0.0;
    if(numberOfBells > 7){ // not sure this is calculated correctly, as it does not take into account an extra beat for open handstroke
        pealTime = (changeInterval * 5000.0);
    } else {
        pealTime = (changeInterval * 5040.0);
    }
    var hours = Math.floor(pealTime/3600.0);
    var minutes = "0" + Math.round((pealTime % 3600)/60.0);
    document.getElementById("CPM").innerHTML=CPM.toFixed(1)+"&nbsp;";
    document.getElementById("CPM").title = "Peal time: " + hours + ":" + minutes.substr(minutes.length-2,2);
    document.getElementById("OH").innerHTML=openHandstroke.toFixed(1)+"&nbsp;";
    if(!nonLive){
        ws.send("BELS:" + numberOfBells + "," + CPM + "," + openHandstroke.toFixed(1) + "," + document.getElementById("chimeSelect").options[document.getElementById("chimeSelect").selectedIndex].text); 
    }
    drawPowerScales();
    foreclick(0); // redraw display with new values

}

function drawSamples(position,iterations){
    var stepSize = (BDwidth * 1.0)/(ROIL-ROIU);
    for (var i = 0; i < iterations; i++){
        var dataEntryOld = sample[position + i -1].slice();
        var dataEntryCurrent = sample[position + i].slice();
        var dataEntryNext = sample[position + i + 1].slice();
        if((currentStatus & RECORDINGSESSION) != 0 || (currentStatus & PLAYBACK) != 0 ) {
            if(dataEntryCurrent[3] == 2) drawLiveTime(dataEntryCurrent[4],false,position+i);
            if(dataEntryCurrent[3] == 1) drawLiveTime(dataEntryCurrent[4],true,position+i);
            drawBell(180-dataEntryCurrent[0]);
        }
//        drawAT(dataEntryCurrent[2]);
        if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] >= 0) { // within ROI for HS1

            if (dataEntryNext[0] >= ROIL) {
                dataEntryCurrent[0] = ROIL; // tidy up at end of ROI
            }
            var startAngle = Math.max(ROIU,dataEntryOld[0]);
            var endAngle = dataEntryCurrent[0];
            if (endAngle <= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTHS1) == 0 ) { // clear highest part of this and previous ROI if this is the first time
                var pixelsFromROIL = (ROIL - startAngle) * stepSize;
                ctxBD.clearRect(posBS2 + pixelsFromROIL, 14, 2 * (BDwidth - pixelsFromROIL), ctxBD.canvas.height - 14 - 25);
                if ((currentStatus & LASTBS2) != 0 ) {
                    drawTimer(position + i, false);
                } else {
                    strokeTimer = 0; // not started yet
                }
                currentStatus &= ~(LASTBS2 | LASTBS1 | LASTHS2);
                currentStatus |= LASTHS1;
            }
            var pixelsFromROIU = (startAngle - ROIU) * stepSize;
            var pixelWidth = (endAngle - startAngle) * stepSize;
            var pixelHeight = (ctxBD.canvas.height - 14 - 25) * Math.min(dataEntryCurrent[2]/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;

            ctxBD.clearRect(posHS1 + pixelsFromROIU, 14, pixelWidth + 1, ctxBD.canvas.height - 14 - 25);
            ctxBD.fillStyle="white";
            ctxBD.fillRect(posHS1 + pixelsFromROIU, ctxBD.canvas.height - 25, pixelWidth, (0-pixelHeight));

        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] >= 0) { // within ROI for HS2
            var endAngle = Math.max(ROIU, 360-dataEntryCurrent[0]);
            var startAngle = Math.min(360-dataEntryOld[0], ROIL);
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTHS2) == 0 ) { // set flags if we have not been before this swing
                currentStatus &= ~(LASTHS1 | LASTBS1 | LASTBS2);
                currentStatus |= LASTHS2;
            }
            var pixelsFromROIL = (ROIL - startAngle) * stepSize;
            var pixelWidth = (startAngle - endAngle) * stepSize;

            var pixelHeight = (ctxBD.canvas.height - 14 - 25) * Math.min(-(dataEntryCurrent[2])/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;

            ctxBD.clearRect(posHS2 + pixelsFromROIL, 14, pixelWidth + 1, ctxBD.canvas.height - 14 - 25);
            ctxBD.fillStyle="white";
            ctxBD.fillRect(posHS2 + pixelsFromROIL, ctxBD.canvas.height - 25, pixelWidth, (0-pixelHeight));
          
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] < 0) { // within ROI for BS1
            if (dataEntryNext[0] <= (360 - ROIL)) {
                dataEntryCurrent[0] = (360 - ROIL); // tidy up at end of ROI
            }
            var startAngle = Math.max(ROIU,360-dataEntryOld[0]);
            var endAngle = Math.min(ROIL,360-dataEntryCurrent[0]);
            if (startAngle >= endAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTBS1) == 0 ) { // clear highest part of this and previous ROI if this is the first time
                var pixelsFromROIL = (ROIL - startAngle) * stepSize;
                ctxBD.clearRect(posHS2 + pixelsFromROIL, 14, 2 * (BDwidth - pixelsFromROIL), ctxBD.canvas.height - 14 - 25);
                if ((currentStatus & LASTHS2) != 0 ) {
                    drawTimer(position + i, false);
                }
                currentStatus &= ~(LASTBS2 | LASTHS1 | LASTHS2);
                currentStatus |= LASTBS1;
            }
            var pixelsFromROIU = (startAngle - ROIU) * stepSize;
            var pixelWidth = (endAngle - startAngle) * stepSize;

            var pixelHeight = (ctxBD.canvas.height - 14 - 25) * Math.min(-(dataEntryCurrent[2])/scaleValue,1)
            if (pixelHeight < 0) pixelHeight = 0;

            ctxBD.clearRect(posBS1 + pixelsFromROIU, 14, pixelWidth + 1, ctxBD.canvas.height - 14 - 25);
            ctxBD.fillStyle="white";
            ctxBD.fillRect(posBS1 + pixelsFromROIU, ctxBD.canvas.height - 25, pixelWidth, (0-pixelHeight));
            
        } else if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] < 0) { // within ROI for BS2
            var endAngle = Math.max(ROIU,dataEntryCurrent[0]);
            var startAngle = Math.min(dataEntryOld[0],ROIL);
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTBS2) == 0 ) { // set flags if we have not been before this swing
                if(currentStatus & LASTHS1) currentStatus |= WOBBLINGATSTAND; // the only way this can happen is with a wobble - this flags stave not to draw anything
                currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2);
                currentStatus |= LASTBS2;
            }
            var pixelsFromROIL = (ROIL - startAngle) * stepSize;
            var pixelWidth = (startAngle - endAngle) * stepSize;

            var pixelHeight = (ctxBD.canvas.height - 14 - 25) * Math.min((dataEntryCurrent[2])/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;

            ctxBD.clearRect(posBS2 + pixelsFromROIL, 14, pixelWidth + 1, ctxBD.canvas.height - 14 - 25);
            ctxBD.fillStyle="white";
            ctxBD.fillRect(posBS2 + pixelsFromROIL, ctxBD.canvas.height - 25, pixelWidth, (0-pixelHeight));
        }
    }
}

function drawLiveTime(time, HS, currentPosition){
    currentATmargin += ATstepWidth;
    if (currentATmargin >= ctxAT.canvas.width/2) currentATmargin -= ctxAT.canvas.width/2;
    ctxAT.clearRect(currentATmargin-20                     ,0,ATstepWidth,ctxAT.canvas.height);
    ctxAT.clearRect(currentATmargin-20+ctxAT.canvas.width/2,0,ATstepWidth,ctxAT.canvas.height);

    var totalHeight = ctxAT.canvas.height-ATbottomMargin-ATtopMargin;
    var range = totalHeight/4.0; //(0.833*totalHeight)/5.0;  //range of +- 2 change "steps"

    var position = 0.0;

    if(HS) {
        position = -range*(time - (changeInterval + (openHandstroke*changeStep)))/changeStep;  // this does a full beat ? should this be half a beat
    } else {
        position = -range*(time - changeInterval)/changeStep;
    }
    var overflow = 0;
    if(position < -totalHeight/2.0){
        overflow = 1;
        position = -totalHeight/2.0;
    }
    if(position > totalHeight/2.0){
        overflow = 1;
        position = totalHeight/2.0
    }

    position += (ATtopMargin + totalHeight/2.0);

    var colour = "white";
    if(overflow) colour = "red";
    
    var height = 0;
    var cross = 0; 
    var j = 0;
    if(switchPoints.length != 0){
        for(j = switchPoints.length - 1; j >= 0 && switchPoints[j] > currentPosition; j--); // find switchpoint preceding the current position
        j = switchPoints[j];
    }
    if (currentPosition - j > 300 || !switchPoints.length){ // 300 = 2.4 seconds worth of samples - not possible to ring this slowly unless really holding up - assume bad data
        height = 10; //default height because the switchPoint is not close enough
        cross = 1;
    } else {
        var i=sample[j][0]; // angle at direction change
        for (k=j; k>0 && Math.abs(sample[k][0] - i) < 5 ;k--);
        var totalPull = 0.0;
        var pull = 0.0;
        for (var m=1;(k+m)<sample.length && Math.abs(sample[k+m][0] - i) < 5; m++){ // add up accns to point 5 degrees forward from direction change
            pull = Math.abs(sample[k+m][2]);
            if(pull < 25) continue;  // ignore stuff below noise floor
            totalPull += pull;
        }
        height = 10 + 25 * ((totalPull/m)/scaleValue);
    }
    if(height > 30) height = 30;
    
    var rise = 0.0;
    if(HS){
        rise=sample[j][0] * (height/10.0); // show a rise of +-10 degrees (consider making this += 20)
    } else {
        rise=(360-sample[j][0]) * (height/10.0);
    }
    if(rise >= height) rise = height;
    if(rise <= -height) rise = -height;
    
    drawBar(currentATmargin,ctxAT,HS,position,height,rise,cross,colour);
    drawBar(currentATmargin+ctxAT.canvas.width/2,ctxAT,HS,position,height,rise,cross,colour);
    
    document.getElementById("canvasAT").style.marginLeft = (currentATmargin * -1) + "px";
}

function drawBar(offset, context,HS,position,height,rise,cross,colour){
    var j = 0;
    context.strokeStyle = colour;
    context.beginPath();
    if(HS){
        context.rect(offset-22, position-height, 18, 2 * height);
        if(cross){
            context.moveTo(offset-22, position-height);
            context.lineTo(offset-4, position+height);
            context.moveTo(offset-4, position-height);
            context.lineTo(offset-22, position+height);
        } else {
            for(j=0;(rise + j) < height; j+=3){
                context.moveTo(offset-22, position+rise+j);
                context.lineTo(offset-4, position+rise+j);
            }
        }
        context.moveTo(offset-24, position-6);
        context.lineTo(offset-24, position+6);
    } else {
        context.rect(offset-24, position-height, 18, 2 * height);
        if(cross){
            context.moveTo(offset-24, position-height);
            context.lineTo(offset-6, position+height);
            context.moveTo(offset-6, position-height);
            context.lineTo(offset-24, position+height);
        } else {
            for(j=0;(rise + j) < height; j+=3){
                context.moveTo(offset-24, position+rise+j);
                context.lineTo(offset-6, position+rise+j);
            }  
        }
        context.moveTo(offset-4, position-6);
        context.lineTo(offset-4, position+6);

    }
    context.stroke();
}

function clearTemplates(){
    ctxBDt.clearRect(0, 0, ctxBDt.canvas.width, ctxBDt.canvas.height);
    ctxBD.clearRect(2*BDwidth , 36, 64, 16);
}

function drawSamplesOnTemplate(position,iterations){
    clearTemplates();

    var stepSize = (BDwidth * 1.0)/(ROIL-ROIU);
    for (var i = 0; i < iterations; i++){
        var dataEntryOld = template[position + i - 1].slice();
        var dataEntryCurrent = template[position + i].slice();
        var dataEntryNext = template[position + i + 1].slice();
        if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] >= 0) { // within ROI for HS1

            if (dataEntryNext[0] >= ROIL) {
                dataEntryCurrent[0] = ROIL; // tidy up at end of ROI
            }
            var startAngle = Math.max(ROIU,dataEntryOld[0]);
            var endAngle = dataEntryCurrent[0];
            if (endAngle <= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            var pixelsFromROIU = (startAngle - ROIU) * stepSize;
            var pixelWidth = (endAngle - startAngle) * stepSize;

            var pixelHeight = (ctxBDt.canvas.height - 14 - 25) * Math.min(dataEntryCurrent[2]/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;
            ctxBDt.fillStyle="rgba(240,240,0,0.6)";
            ctxBDt.fillRect(posHS1 + pixelsFromROIU, ctxBDt.canvas.height - 25, pixelWidth, (0-pixelHeight));

        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] >= 0) { // within ROI for HS2
            var endAngle = Math.max(ROIU, 360-dataEntryCurrent[0]);
            var startAngle = Math.min(360-dataEntryOld[0], ROIL);
            
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            var pixelsFromROIL = (ROIL - startAngle) * stepSize;
            var pixelWidth = (startAngle - endAngle) * stepSize;

            var pixelHeight = (ctxBDt.canvas.height - 14 - 25) * Math.min(-(dataEntryCurrent[2])/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;

            ctxBDt.fillStyle="rgba(240,240,0,0.6)";
            ctxBDt.fillRect(posHS2 + pixelsFromROIL, ctxBDt.canvas.height - 25, pixelWidth, (0-pixelHeight));
            
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] < 0) { // within ROI for BS1

            if (dataEntryNext[0] <= (360 - ROIL)) {
                dataEntryCurrent[0] = (360 - ROIL); // tidy up at end of ROI
            }
            var startAngle = Math.max(ROIU,360-dataEntryOld[0]);
            var endAngle = Math.min(ROIL,360-dataEntryCurrent[0]);
            
            if (startAngle >= endAngle) {
                continue; // for moment don't swap, just do nothing
            }
            var pixelsFromROIU = (startAngle - ROIU) * stepSize;
            var pixelWidth = (endAngle - startAngle) * stepSize;

            var pixelHeight = (ctxBDt.canvas.height - 14 - 25) * Math.min(-(dataEntryCurrent[2])/scaleValue,1)
            if (pixelHeight < 0) pixelHeight = 0;
            ctxBDt.fillStyle="rgba(240,240,0,0.6)";
            ctxBDt.fillRect(posBS1 + pixelsFromROIU, ctxBDt.canvas.height - 25, pixelWidth, (0-pixelHeight));
       
        } else if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] < 0) { // within ROI for BS2
            var endAngle = Math.max(ROIU,dataEntryCurrent[0]);
            var startAngle = Math.min(dataEntryOld[0],ROIL);
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            var pixelsFromROIL = (ROIL - startAngle) * stepSize;
            var pixelWidth = (startAngle - endAngle) * stepSize;

            var pixelHeight = (ctxBDt.canvas.height - 14 - 25) * Math.min((dataEntryCurrent[2])/scaleValue,1);
            if (pixelHeight < 0) pixelHeight = 0;
            ctxBDt.fillStyle="rgba(240,240,0,0.6)";
            ctxBDt.fillRect(posBS2 + pixelsFromROIL, ctxBDt.canvas.height - 25, pixelWidth, (0-pixelHeight));
        }
    }
    drawPowerScales();  // function checks what needs to be drawn on template like gridlines and target angles
}

function setStatus(text){
    document.getElementById("statusLine").innerHTML=text;
    if (statusintervalID != null) clearInterval(statusintervalID);
    statusintervalID=setInterval(clearStatus,5000); // display status for 5 seconds
}

function clearStatus() {
    document.getElementById("statusLine").innerHTML="";
    clearInterval(statusintervalID);
    statusintervalID=null;
}

////////////////////////////////////////////////////////////////
//                   BUTTON CLICK FUNCTIONS                   //
////////////////////////////////////////////////////////////////

fileOpenButton.onclick = function() {
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    var e = document.getElementById("openSelect");
    sample = [];
    ringTimes = [];
    switchPoints = [];
    ws.send("LOAD:" + e.options[e.selectedIndex].text);
    openModal.style.display = "none";
    currentStatus |= DOWNLOADINGFILE;
    currentStatus &= ~SESSIONLOADED;
    currentSwingDisplayed = null;
    clearBell();
    ctxBD.clearRect(posCB+1, ctxBD.canvas.height-30, CBwidth-2, 20); // clear existing timers
    updateIcons();
};

document.getElementById('autoButton').onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) {
        setStatus("A session must be loaded to do this.");
        return;
    }

    var BstrikeCount = 0;
    var HstrikeCount = 0;
    var averageBSTime = 0.0;
    var averageHSTime = 0.0;
    var impliedOHFactor = 0.0;
    var HditchFirst = 0;
    var BditchFirst = 0;

    for(i=0;i<sample.length;++i){
        if(sample[i][3] == 1){
            if(HditchFirst != 0){
                HstrikeCount += 1;
                averageHSTime = ((averageHSTime * (HstrikeCount - 1)) + sample[i][4])/HstrikeCount;
            } else {
                HditchFirst = 1;
            }

        } else if(sample[i][3] == 2){
            if(BditchFirst != 0){
                BstrikeCount += 1;
                averageBSTime = ((averageBSTime * (BstrikeCount - 1)) + sample[i][4])/BstrikeCount;
            } else {
                BditchFirst = 1;
            }
        }
    }
    calculateTimings();
    var impliedCPM = 60.0/averageBSTime;
    var impliedOH = (averageHSTime-averageBSTime)/changeStep;
    if(impliedOH < 0.0) impliedOH = 0.0;
    if(impliedOH > 2.2) impliedOH = 2.2;
    if(CPM < 20.0) CPM = 20.0;
    if(CPM > 50.0) CPM = 50.0;

    if (confirm("About to set CPM at " + impliedCPM.toFixed(1) + " and handstroke factor at " + impliedOH.toFixed(1) + ". Does this look sane?")) {
        CPM = impliedCPM;
        openHandstroke = impliedOH;
        calculateTimings();
    }
};

calibButton.onclick = function() {
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if (!wsOpened){
        setStatus("No communication with BellBoy device.");
        return;
    }
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    demoModal.style.display = "block";
    settingsModal.style.display = "none";
    ws.send("EYEC:");
};

plusCPMButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if(CPM < 50) CPM += 0.5;
    calculateTimings();
}

minusCPMButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if(CPM > 20) CPM -= 0.5;
    calculateTimings();
};

plusSleepButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if(sleepTime < 24.0) sleepTime += 0.5;
    document.getElementById("sleep").innerHTML=sleepTime.toFixed(1)+"&nbsp;";
}

minusSleepButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if(sleepTime > 0) sleepTime -= 0.5;
    document.getElementById("sleep").innerHTML=sleepTime.toFixed(1)+"&nbsp;";
};

sleepButton.onclick = function() {
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if (!wsOpened){
        setStatus("No communication with BellBoy device.");
        return;
    }
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (confirm("Are you sure you want to send me to sleep?")) ws.send("SLEP:" + (sleepTime*2));
};

plusOHButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (openHandstroke < 2.19) openHandstroke += 0.1;
    calculateTimings();
};

minusOHButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (openHandstroke > 0.11) openHandstroke -= 0.1;
    calculateTimings();
};

diagnosticsButton.onclick = function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) {
        setStatus("A session must be loaded for diagnostics.");
        return;
    }
    settingsModal.style.display = "none";
    diagnosticsModal.style.display = "block";
    fillDiagnosticsModal();
};

document.getElementById('diagHand').onclick = function(){
    fillDiagnosticsModal();
};

document.getElementById('diagBack').onclick = function(){
    fillDiagnosticsModal();
};

function fillDiagnosticsModal(){
    var diagnosticsCanvas = document.getElementById("canvasDiagnostics");
    var ctxDiagnostics = diagnosticsCanvas.getContext("2d");
    var i;
    diagnosticsCanvas.style.width= "800 px";
    diagnosticsCanvas.style.height="325 px";
    diagnosticsCanvas.width= 800;
    diagnosticsCanvas.height=325;
    ctxDiagnostics.canvas.width=800;
    ctxDiagnostics.canvas.height=325;
    ctxDiagnostics.clearRect(0, 0, ctxDiagnostics.canvas.width, ctxDiagnostics.canvas.height);

    var midHeight = 150;

    ctxDiagnostics.beginPath();
    ctxDiagnostics.strokeStyle = "blue";

    if(document.getElementById('diagHand').checked && document.getElementById('diagBack').checked){
        ctxDiagnostics.moveTo(40+(sample[0][0])*2,-(sample[0][2])*0.5+midHeight);
        for(i=1;i<sample.length;++i){
            ctxDiagnostics.lineTo(40+(sample[i][0])*2,-(sample[i][2])*0.5+midHeight);
        }
    }

    if(document.getElementById('diagHand').checked && !document.getElementById('diagBack').checked){
        i=0;
        while(i<sample.length){
            for(; i<sample.length && sample[i][1] <= 0; ++i);
            if(i == sample.length) break;
            ctxDiagnostics.moveTo(40+(sample[i][0])*2,-(sample[i][2])*0.5+midHeight);
            for(; i<sample.length && sample[i][1] > 0; ++i) ctxDiagnostics.lineTo(40+(sample[i][0])*2,-(sample[i][2])*0.5+midHeight);
        }
    }

    if(!document.getElementById('diagHand').checked && document.getElementById('diagBack').checked){
        i=0;
        while(i<sample.length){
            for(; i<sample.length && sample[i][1] >= 0; ++i);
            if(i == sample.length) break;
            ctxDiagnostics.moveTo(40+(sample[i][0])*2,-(sample[i][2])*0.5+midHeight);
            for(; i<sample.length && sample[i][1] < 0; ++i) ctxDiagnostics.lineTo(40+(sample[i][0])*2,-(sample[i][2])*0.5+midHeight);
        }
    }

    ctxDiagnostics.stroke();
    for(i=0;i<sample.length;++i){
        if(sample[i][3] == 1){
            if(!document.getElementById('diagHand').checked) continue;
            ctxDiagnostics.beginPath();
            ctxDiagnostics.strokeStyle = "green";
            ctxDiagnostics.moveTo(40+(sample[i][0])*2,180);
            ctxDiagnostics.lineTo(35+(sample[i][0])*2,175);
            ctxDiagnostics.lineTo(45+(sample[i][0])*2,175);
            ctxDiagnostics.lineTo(40+(sample[i][0])*2,180);
            ctxDiagnostics.stroke();
        } else if(sample[i][3] == 2){
            if(!document.getElementById('diagBack').checked) continue;
            ctxDiagnostics.beginPath();
            ctxDiagnostics.strokeStyle = "green";
            ctxDiagnostics.moveTo(40+(sample[i][0])*2,120);
            ctxDiagnostics.lineTo(35+(sample[i][0])*2,125);
            ctxDiagnostics.lineTo(45+(sample[i][0])*2,125);
            ctxDiagnostics.lineTo(40+(sample[i][0])*2,120);
            ctxDiagnostics.stroke();
        }
    }

    ctxDiagnostics.beginPath();
    ctxDiagnostics.strokeStyle = "black";
    ctxDiagnostics.rect(0, 0, 800, 300);
    ctxDiagnostics.moveTo(40, 0);
    ctxDiagnostics.lineTo(40, 310);
    ctxDiagnostics.moveTo(400, 0);
    ctxDiagnostics.lineTo(400, 310);
    ctxDiagnostics.moveTo(760, 0);
    ctxDiagnostics.lineTo(760, 310);
    ctxDiagnostics.moveTo(0, 150);
    ctxDiagnostics.lineTo(799, 150);
    ctxDiagnostics.stroke();
    ctxDiagnostics.font = "12px sans serif";
    ctxDiagnostics.fillStyle = "black";
    ctxDiagnostics.textAlign = "center";
    ctxDiagnostics.textBaseline="top";
    ctxDiagnostics.fillText("0", 40, 311);
    ctxDiagnostics.fillText("180", 400, 311);
    ctxDiagnostics.fillText("360", 760, 311);
    
    var BstrikeCount = 0;
    var HstrikeCount = 0;
    var averageBSTime = 0.0;
    var averageHSTime = 0.0;
    var impliedOHFactor = 0.0;
    var HditchFirst = 0;
    var BditchFirst = 0;
    var lastBDC = 0;
    var averageHSStrikeAngle = 0.0;
    var averageBSStrikeAngle = 0.0;
    var averageBSStrikeTimeFromBDC = 0.0;
    var averageHSStrikeTimeFromBDC = 0.0;

    for(i=0;i<sample.length;++i){
        if (sample[i][3] == 8) lastBDC = i;
        if(sample[i][3] == 1){
            if(HditchFirst != 0){
                HstrikeCount += 1;
                averageHSTime = ((averageHSTime * (HstrikeCount - 1)) + sample[i][4])/HstrikeCount;
                averageHSStrikeAngle = ((averageHSStrikeAngle * (HstrikeCount - 1)) + sample[i][0])/HstrikeCount;
                averageHSStrikeTimeFromBDC = ((averageHSStrikeTimeFromBDC * (HstrikeCount - 1)) + ((i-lastBDC)*0.008))/HstrikeCount;
            } else {
                HditchFirst = 1;
            }

        } else if(sample[i][3] == 2){
            if(BditchFirst != 0){
                BstrikeCount += 1;
                averageBSTime = ((averageBSTime * (BstrikeCount - 1)) + sample[i][4])/BstrikeCount;
                averageBSStrikeAngle = ((averageBSStrikeAngle * (BstrikeCount - 1)) + sample[i][0])/BstrikeCount;
                averageBSStrikeTimeFromBDC = ((averageBSStrikeTimeFromBDC * (BstrikeCount - 1)) + ((i-lastBDC)*0.008))/BstrikeCount;
            } else {
                BditchFirst = 1;
            }
        }
    }
    calculateTimings(); // should have already been updated
    document.getElementById("gravityValue").innerHTML= "Gravity: " + gravityValue.toString() + " Rope(H): " + ropeValueH.toString() + " Rope(B): " + ropeValueB.toString();
    document.getElementById("impliedTimings").innerHTML= "CPM: " + (60.0/averageBSTime).toFixed(1) + " Open Handstroke Factor: " + ((averageHSTime-averageBSTime)/changeStep).toFixed(1);
    var pealTime = 0.0;
    if((document.getElementById("bellsSelect").selectedIndex + 3) > 7){
        pealTime = (changeInterval * 5000.0);
    } else {
        pealTime = (changeInterval * 5040.0);
    }
    var hours = Math.floor(pealTime/3600.0);
    var minutes = "0" + Math.round((pealTime % 3600)/60.0);
    
    document.getElementById("impliedTimings").title = "Peal time: " + hours + ":" + minutes.substr(minutes.length-2,2);
    document.getElementById("averageStrikeAngles").innerHTML= "Handstroke: " + averageHSStrikeAngle.toFixed(0) + " Backstroke: " + averageBSStrikeAngle.toFixed(0);
    document.getElementById("averageStrikeTimes").innerHTML= "Handstroke: " + (averageHSStrikeTimeFromBDC*1000).toFixed(0) + " Backstroke: " + (averageBSStrikeTimeFromBDC * 1000).toFixed(0);
}

playbackButton.onclick = function() {
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if (!wsOpened){
        setStatus("No communication with BellBoy device.");
        return;
    }
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) return;

    if (this.innerText == "Playback only"){
        this.innerText = "Play and record";
        ws.send("STND:");
    } else {
        this.innerText = "Playback only";
        ws.send("PLRD:");
    }
};

bellsSelect.onchange = function() {
    calculateTimings();
};

chimeSelect.onchange = function() {
    calculateTimings();
};

targetSelectHand.onchange = function() {
    var t = this.options[this.selectedIndex].text
    if (t.isNaN) {
        targetAngleHand=null
    } else {
        targetAngleHand=parseInt(t)
    }
    if (currentStatus & TEMPLATEDISPLAYED){
        drawStrokeT();
    } else {
        clearTemplates();
        drawPowerScales();
    }
};

targetSelectBack.onchange = function() {
    var t = this.options[this.selectedIndex].text
    if (t.isNaN) {
        targetAngleBack=null
    } else {
        targetAngleBack=parseInt(t)
    }
    if (currentStatus & TEMPLATEDISPLAYED){
        drawStrokeT();
    } else {
        clearTemplates();
        drawPowerScales();
    }
};

zoomSelect.onchange = function(){
    currentROI = this.selectedIndex;
    ROIU = ROIRanges[currentROI][0];
    ROIL = ROIRanges[currentROI][1];
    drawPowerFrame();
    if (currentStatus & TEMPLATEDISPLAYED) {
        drawStrokeT();
    } else {
        clearTemplates();
        drawPowerScales();
    }
    
    if (currentSwingDisplayed == null) return;
    drawStroke();
    textBell();
};

speedSelect.onchange = function(){
    currentPlaybackSpeed = this.selectedIndex;
// consider whether to allow this to be changed during active playback
};

scaleSelect.onchange = function() {
    scaleValue=parseFloat(scales[this.selectedIndex])
    drawPowerFrame();
    clearStave();
    if (currentStatus & TEMPLATEDISPLAYED) {
        drawStrokeT();
    } else {
        clearTemplates();
        drawPowerScales();
    }
    if (currentSwingDisplayed == null) return;
    drawStroke();
    textBell();
};

recordIcon.onclick=function(){
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if (!wsOpened){
        setStatus("No communication with BellBoy device.  Aborted.");
        return;
    }
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & RECORDINGSESSION) {
        currentStatus &= ~RECORDINGSESSION;
        updateIcons();
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        clearBell();
        currentPlaybackPosition=1;
        ws.send("STOP:");
    } else {
        document.getElementById("recordFileName").value = "";
        document.getElementById("nameInvalid").style.visibility = "hidden";
        recordModal.style.display = "block";
        document.getElementById("recordFileName").focus();
    }
};

recordButton.onclick = function() {
    var fileName = document.getElementById("recordFileName").value;
    var patt = /^[a-z0-9_. ()-]+$/i;
    if ((patt.test(fileName) && fileName.length < 31) || fileName == "") {
        recordModal.style.display = "none";
        currentStatus |= RECORDINGSESSION;
        currentStatus &= ~SESSIONLOADED;
        updateIcons();
        drawPowerFrame();
        sample = [];
        ringTimes = [];
        switchPoints = [];
        clearStave();
        currentPlaybackPosition = 1; // needs to be one as we are doing comparison with previous entry removed so that playback starts from last swing displayed
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=setInterval(showLive,collectInterval*1000);
        currentSwingDisplayed=null;
        ws.send("STRT:" + fileName);
    } else {
        document.getElementById("nameInvalid").style.visibility = "visible";
    }
};

// to do, when PLAYED THEN stopped scaling does not work
// set template displayed so we know to scale template

powerIcon.onclick = function() {
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if (!wsOpened){
        setStatus("No communication with BellBoy device.  Aborted.");
        return;
    }
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (confirm("Are you sure you want to power off?")) ws.send("SHDN:");
    return;
};

playIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) return;
    if (currentStatus & HELPDISPLAYED) return;

    if (currentStatus & PLAYBACK) {
        currentStatus &= ~PLAYBACK;
        currentStatus &= ~PAUSED;
        updateIcons();
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        clearBell();
        currentPlaybackPosition=1;
    } else {
        currentStatus |= PLAYBACK;
        currentStatus &= ~PAUSED;
        updateIcons();
        drawPowerFrame();
        clearStave();
//        currentPlaybackPosition =1; // needs to be one as we are doing comparison with previous entry removed so that playback starts from last swing displayed
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,collectInterval*1000);
        currentSwingDisplayed=null;
        ctxBD.clearRect(posCB+1, ctxBD.canvas.height-30, CBwidth-2, 20); // clear existing timers
    }
};

pauseIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (!(currentStatus & PLAYBACK)) return;

    if (currentStatus & PAUSED) {
        currentStatus &= ~PAUSED;
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,collectInterval*1000);
        textBell();
    } else {
        currentStatus |= PAUSED;
        if (playintervalID != null) clearInterval(playintervalID);
        textBell();
    }
};

skipSelectIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & SKIPMAIN){
        if(currentStatus & SKIPTEMPLATE){
            currentStatus &= ~SKIPTEMPLATE;
        } else {
            if(currentStatus & TEMPLATEDISPLAYED){
                currentStatus |= SKIPTEMPLATE;
                currentStatus &= ~SKIPMAIN;
            }
        }
    } else {
        currentStatus |= SKIPMAIN;
    }
    updateIcons();
};

// supposed to stop zooming - does not appear to work
forwardIcon.addEventListener("touchstart", preventZoom);
backIcon.addEventListener("touchstart", preventZoom);

forwardIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & PLAYBACK) return;
    foreclick(1);
};

backIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & PLAYBACK) return;

    backclick(1);
};

function foreclick(skip) {
    if ((currentStatus & TEMPLATEDISPLAYED) && (currentStatus & SKIPTEMPLATE) && (currentStatus & SKIPMAIN) && (TcurrentSwingDisplayed+skip > TswingStarts.length -1 || currentSwingDisplayed+skip > swingStarts.length - 1)) return;

    if ((currentStatus & SESSIONLOADED) && (currentStatus & SKIPMAIN)  && (currentSwingDisplayed+skip <= swingStarts.length - 1)) {
        if (currentSwingDisplayed == null) currentSwingDisplayed=-skip;
        currentSwingDisplayed += skip;
        drawStroke();
        drawStrokeT();
    }

    if ((currentStatus & TEMPLATEDISPLAYED) && (currentStatus & SKIPTEMPLATE) &&  (TcurrentSwingDisplayed+skip <= TswingStarts.length -1)) {
        if (TcurrentSwingDisplayed == null) TcurrentSwingDisplayed=-skip;
        TcurrentSwingDisplayed += skip;
        drawStroke();
        drawStrokeT();
    }
    textBell();
    updateIcons();
};

function backclick(skip){
    if ((currentStatus & TEMPLATEDISPLAYED) && (currentStatus & SKIPTEMPLATE) && (currentStatus & SKIPMAIN) && (TcurrentSwingDisplayed-skip < 0 || currentSwingDisplayed-skip < 0)) return;

    if ((currentStatus & SESSIONLOADED) && (currentStatus & SKIPMAIN)  && (currentSwingDisplayed-skip >= 0)) {
        if (currentSwingDisplayed == null) currentSwingDisplayed=skip;
        currentSwingDisplayed -= skip;
        drawStroke();
        drawStrokeT();
    }

    if ((currentStatus & TEMPLATEDISPLAYED) && (currentStatus & SKIPTEMPLATE)  && (TcurrentSwingDisplayed-skip >= 0)) {
        if (TcurrentSwingDisplayed == null) TcurrentSwingDisplayed=skip;
        TcurrentSwingDisplayed -= skip;
        drawStroke();
        drawStrokeT();
    }

    textBell();
    updateIcons();
};

favIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & PLAYBACK) return;
    if (currentSwingDisplayed == null) return;

    template=[];
    TswingStarts=[];
    TpullStrengths=[];
    TringTimes=[]
    TswitchPoints=[]
    
    clearTemplates();
    drawPowerScales();
    var i = 0;
    for (i = 0; i<sample.length; ++i) template[template.length]=sample[i].slice();
    for (i = 0; i<swingStarts.length; ++i) TswingStarts[TswingStarts.length]=swingStarts[i];
    for (i = 0; i<pullStrengths.length; ++i) TpullStrengths[TpullStrengths.length]=pullStrengths[i];
    for (i = 0; i<ringTimes.length; ++i) TringTimes[TringTimes.length]=ringTimes[i];
    for (i = 0; i<switchPoints.length; ++i) TswitchPoints[TswitchPoints.length]=switchPoints[i];

    TchangeInterval = changeInterval;
    TopenHandstroke = openHandstroke;
    TchangeStep= changeStep;

    TcurrentSwingDisplayed=currentSwingDisplayed;
    currentStatus |= TEMPLATEDISPLAYED;
    foreclick(0);
    textBell();    
    updateIcons();
};

unstarIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentStatus & PLAYBACK) return;

    template=[];
    TswingStarts=[];
    TcurrentSwingDisplayed=null;
    TpullStrengths = [];
    clearTemplates();
    currentStatus &= ~TEMPLATEDISPLAYED;
    currentStatus &= ~SKIPTEMPLATE;
    currentStatus |= SKIPMAIN;
    drawPowerScales();
    ctxBD.clearRect(posCB+1, ctxBD.canvas.height-60, CBwidth-2, 20);  // clear timer displays
    drawStroke();
    updateIcons();
};

helpIcon.onclick=function(){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & PLAYBACK) return;
    if (currentStatus & RECORDINGSESSION) return;

    if (currentStatus & HELPDISPLAYED) {
        currentStatus &= ~HELPDISPLAYED;
        document.getElementById("bellGraphics").style.display = "block";
        document.getElementById("helpScreen").style.display = "none";
    } else {
        currentStatus |= HELPDISPLAYED;
        document.getElementById("bellGraphics").style.display = "none";
        document.getElementById("helpScreen").style.display = "block";
    }
    updateIcons();
};

function updateIcons(){
    // recordIcon
    if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & PLAYBACK) ||
        (currentStatus & HELPDISPLAYED)) {
        document.getElementById("recordIcon").src = "circle-record-inactive.png";
    } else {
        if (currentStatus & RECORDINGSESSION) {
            document.getElementById("recordIcon").src = "circle-stop-active.png";
        } else {
            document.getElementById("recordIcon").src = "circle-record-active.png";
        }
    }

    // powerIcon
    if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & PLAYBACK) ||
        (currentStatus & RECORDINGSESSION)) {
        document.getElementById("powerIcon").src = "circle-power-inactive.png";
    } else {
        document.getElementById("powerIcon").src = "circle-power-active.png";
    }

    //downloadIcon
    if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & PLAYBACK) ||
        (currentStatus & RECORDINGSESSION) ||
        (currentStatus & HELPDISPLAYED)) {
        document.getElementById("downloadIcon").src = "cloud-download-inactive.png";
    } else {
        document.getElementById("downloadIcon").src = "cloud-download-active.png";
    }

    //playIcon
    if (!(currentStatus & SESSIONLOADED)) {
        document.getElementById("playIcon").src = "circle-play-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) ||
            (currentStatus & HELPDISPLAYED)) {
            document.getElementById("playIcon").src = "circle-play-inactive.png";
        } else {
            if (currentStatus & PLAYBACK) {
                document.getElementById("playIcon").src = "circle-stop-active.png";
            } else {
                document.getElementById("playIcon").src = "circle-play-active.png";
            }
        }
    }

    //pauseIcon
    if (!(currentStatus & PLAYBACK)) {
        document.getElementById("pauseIcon").src = "circle-pause-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) ||
            (currentStatus & HELPDISPLAYED)) {
            document.getElementById("pauseIcon").src = "circle-pause-inactive.png";
        } else {
                document.getElementById("pauseIcon").src = "circle-pause-active.png";
        }
    }

    //skipIcons and fav icons
    if (!(currentStatus & SESSIONLOADED)) {
        document.getElementById("backIcon").src = "circle-back-inactive.png";
        document.getElementById("forwardIcon").src = "circle-forward-inactive.png";
        document.getElementById("favIcon").src = "circle-heart-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) ||
            (currentStatus & PLAYBACK) ||
            (currentStatus & HELPDISPLAYED)) {
            document.getElementById("backIcon").src = "circle-back-inactive.png";
            document.getElementById("forwardIcon").src = "circle-forward-inactive.png";
            document.getElementById("favIcon").src = "circle-heart-inactive.png";
        } else {
            if (currentSwingDisplayed == swingStarts.length - 1) {
                document.getElementById("forwardIcon").src = "circle-forward-inactive.png";
            } else {
                document.getElementById("forwardIcon").src = "circle-forward-active.png";
            }
            if (currentSwingDisplayed == 0) {
                document.getElementById("backIcon").src = "circle-back-inactive.png";
            } else {
                document.getElementById("backIcon").src = "circle-back-active.png";
            }
            if (currentSwingDisplayed != null) {
                document.getElementById("favIcon").src = "circle-heart-active.png";
            } else {
                document.getElementById("favIcon").src = "circle-heart-inactive.png";
            }
        }
    }
    
    
    // skip select icon
    if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & PLAYBACK) ||
        !(currentStatus & SESSIONLOADED) ||
        (currentStatus & HELPDISPLAYED)) {
        document.getElementById("skipSelectIcon").src = "skip-select-inactive.png";
    } else {
        if(currentStatus & SKIPMAIN){
            if(currentStatus & SKIPTEMPLATE) {
                document.getElementById("skipSelectIcon").src = "skip-select-active-both.png";
            } else {
                document.getElementById("skipSelectIcon").src = "skip-select-active-main.png";
            }
        } else {
            document.getElementById("skipSelectIcon").src = "skip-select-active-template.png";
        }
    }

    // helpIcon
    if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & RECORDINGSESSION) ||
        (currentStatus & PLAYBACK)) {
        document.getElementById("helpIcon").src = "help-inactive.png";
    } else {
        if (currentStatus & HELPDISPLAYED) {
            document.getElementById("helpIcon").src = "door-out.png";
        } else {
            document.getElementById("helpIcon").src = "help.png";
        }
    }

    //remove template icon
    if ((currentStatus & DOWNLOADINGFILE) ||
        ((currentStatus & PLAYBACK) && !(currentStatus & PAUSED)) ||
        (currentStatus & HELPDISPLAYED)) {
        document.getElementById("unstarIcon").src = "circle-cross-inactive.png";

    } else {
        if (currentStatus & TEMPLATEDISPLAYED){
            document.getElementById("unstarIcon").src = "circle-cross-active.png";
        } else {
            document.getElementById("unstarIcon").src = "circle-cross-inactive.png";
        }
    }

    // settingsicon and timingsicon
       if ((currentStatus & DOWNLOADINGFILE) ||
        (currentStatus & PLAYBACK) ||
        (currentStatus & RECORDINGSESSION) ||
        (currentStatus & HELPDISPLAYED)) {
        document.getElementById("settingsIcon").src = "settings-inactive.png";
        document.getElementById("timingsIcon").src = "timings-inactive.png";
    } else {
        document.getElementById("settingsIcon").src = "settings-active.png";
        document.getElementById("timingsIcon").src = "timings-active.png";
    }

}
////////////////////////////////////////////////////////////////
//                       MODAL FUNCTIONS                      //
////////////////////////////////////////////////////////////////


var openModal = document.getElementById("openModal");
var openBtn = document.getElementById("downloadIcon");
var openSpan = document.getElementsByClassName("close")[0];

var recordModal = document.getElementById("recordModal");
var recordSpan = document.getElementsByClassName("close")[1];

var diagnosticsModal = document.getElementById("diagnosticsModal");
var diagnosticsSpan = document.getElementsByClassName("close")[2];

var settingsModal = document.getElementById("settingsModal");
var settingsBtn = document.getElementById("settingsIcon");
var settingsSpan = document.getElementsByClassName("close")[3];

var demoModal = document.getElementById("demoModal");
var demoSpan = document.getElementsByClassName("close")[4];

var timingsModal = document.getElementById("timingsModal");
var timingsBtn = document.getElementById("timingsIcon");
var timingsSpan = document.getElementsByClassName("close")[5];

openBtn.onclick = function() {
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    openModal.style.display = "block";
};

settingsBtn.onclick = function() {
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    settingsModal.style.display = "block";
    recalculateSize();// useful for ipad resizing
    drawPowerScales();
    foreclick(0);
};

timingsBtn.onclick = function() {
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    timingsModal.style.display = "block";
};

// record button onclick is is main onclick area

// When the user clicks on <span> (x), close the modal
openSpan.onclick = function() {
    openModal.style.display = "none";
};

settingsSpan.onclick = function() {
    settingsModal.style.display = "none";
};

recordSpan.onclick = function() {
    recordModal.style.display = "none";
};

diagnosticsSpan.onclick = function() {
    diagnosticsModal.style.display = "none";
};

demoSpan.onclick = function() {
    demoModal.style.display = "none";
    ws.send("STEC:");
};

timingsSpan.onclick = function() {
    timingsModal.style.display = "none";
};

// When the user clicks anywhere outside of the modal, close it
window.onclick = function(event) {
    if (event.target == openModal) {
        openModal.style.display = "none";
    }
    if (event.target == settingsModal) {
        settingsModal.style.display = "none";
    }
    if (event.target == recordModal) {
        recordModal.style.display = "none";
    }
    if (event.target == diagnosticsModal) {
        diagnosticsModal.style.display = "none";
    }
    if (event.target == demoModal) {
        demoModal.style.display = "none";
        ws.send("STEC:");
    }
    if (event.target == timingsModal) {
        timingsModal.style.display = "none";
    }
};

////////////////////////////////////////////////////////////////
//           WINDOW RESIZE / CANVAS CLICK FUNCTIONS           //
////////////////////////////////////////////////////////////////

window.addEventListener("resize", function(event){
    recalculateSize();
    if (currentStatus & TEMPLATEDISPLAYED) drawStrokeT();
    drawPowerScales();
    if (currentStatus & SESSIONLOADED) drawStroke();
});

window.addEventListener("load", function(event){
    recalculateSize();
    updateIcons();
    drawPowerScales();
    clearStave();
    document.body.addEventListener("touchstart", preventZoom); // doesn't appear to work
});

canvasATt.addEventListener("click", function(event){
    if (currentStatus & DOWNLOADINGFILE) return;
    if (currentStatus & RECORDINGSESSION) return;
    if (!(currentStatus & SESSIONLOADED)) return;
    if (currentStatus & HELPDISPLAYED) return;
    if (currentSwingDisplayed == null) return;
    var dist = Math.round((event.pageX - ctxATt.canvas.width/2.0)/(ATstepWidth*2));
    if(dist<0){
        backclick(-dist);
    } else {
        foreclick(dist);
    }
},false);


function recalculateSize() {
    var winWidth =  window.innerWidth;
    var rightWidth = Math.max(800,winWidth-16);

    var canvasWidth=rightWidth-16;
    var canvasHeight=((rightWidth-CBwidth)/4)/1.3;

    var winHeight =  window.innerHeight;
    var rightHeight = winHeight - 210;

    document.getElementById("wrapper").style.width = winWidth; // corrects iPad resizing

    document.getElementById("bellGraphics").style.width = rightWidth + "px";
    document.getElementById("bellGraphics").style.height = rightHeight + "px";
    document.getElementById("helpScreen").style.width = rightWidth + "px";
    document.getElementById("helpScreen").style.height = rightHeight + "px";
    document.getElementById("helpIframe").style.width = (rightWidth-42) + "px";
    document.getElementById("helpIframe").style.height = (rightHeight -20) + "px";

    canvasBD.style.width=canvasWidth  + "px";
    canvasBD.style.height=canvasHeight  + "px";
    ctxBD.canvas.width=canvasWidth;
    ctxBD.canvas.height=canvasHeight;
    canvasBD.style.top=10 + "px";
    canvasBD.style.left=8 + "px";

    canvasBDt.style.width=canvasWidth + "px";
    canvasBDt.style.height=canvasHeight + "px";
    ctxBDt.canvas.width=canvasWidth;
    ctxBDt.canvas.height=canvasHeight;
    canvasBDt.style.top=10 + "px";
    canvasBDt.style.left=8 + "px";

    posBS2 = 0;
    BDwidth = (canvasWidth-CBwidth)/4 
    posHS1 = BDwidth;
    posCB = BDwidth * 2;
    posHS2 = canvasWidth - (BDwidth *2);
    posBS1 = canvasWidth - BDwidth;

    canvasAT.style.width=(rightWidth * 2)  + "px";  // scrolling display
    canvasAT.style.height=(rightHeight-canvasHeight-40)  + "px";
    ctxAT.canvas.width=rightWidth * 2;
    ctxAT.canvas.height=rightHeight-canvasHeight-40;
    canvasAT.style.top=(canvasHeight+30)  + "px";
    canvasAT.style.left=0 + "px";

    canvasATt.style.width=rightWidth + "px";
    canvasATt.style.height=(rightHeight-canvasHeight-40)  + "px";
    ctxATt.canvas.width=rightWidth;
    ctxATt.canvas.height=rightHeight-canvasHeight-40;
    canvasATt.style.top=(canvasHeight+30) + "px";
    canvasATt.style.left=0 + "px";
   
    ctxBD.fillStyle='rgba(0,0,255,0.1)';
    ctxBD.fillRect(posBS2,0,BDwidth,canvasBD.height);
    ctxBD.fillStyle='rgba(0,255,255,0.1)';
    ctxBD.fillRect(posHS1,0,BDwidth,canvasBD.height);
    ctxBD.fillStyle='rgba(255,0,255,0.1)';
    ctxBD.fillRect(posHS2,0,BDwidth,canvasBD.height);
    ctxBD.fillStyle='rgba(255,255,255,0.5)';
    ctxBD.fillRect(posBS1,0,BDwidth,canvasBD.height);

    drawPowerFrame();
    clearStave();
    
}

function drawPowerFrame(){
    ctxBD.clearRect(posBS2, 0, BDwidth * 2, ctxBD.canvas.height);
    ctxBD.clearRect(posHS2, 0, BDwidth * 2, ctxBD.canvas.height);
    ctxBD.lineWidth=1;
    ctxBD.font = "12px sans serif";
    ctxBD.fillStyle = "white";
    ctxBD.textAlign = "center";
    ctxBD.fillText("Handstroke Pull", posHS1+ BDwidth/2, 10);
    ctxBD.fillText("Backstroke Check", posHS2 + BDwidth/2, 10);
    ctxBD.fillText("Backstroke Pull", posBS1 + BDwidth/2, 10);
    ctxBD.fillText("Handstroke Check", BDwidth/2, 10);

    ctxBD.beginPath();
    ctxBD.strokeStyle="rgba(240,240,240,0.6)";
    ctxBD.rect(posBS2, -1, BDwidth, 13);
    ctxBD.rect(posHS1, -1, BDwidth, 13);
    ctxBD.rect(posHS2, -1, BDwidth, 13);
    ctxBD.rect(posBS1, -1, BDwidth, 13);
    ctxBD.rect(posBS2, ctxBD.canvas.height-24, BDwidth * 2, 25);
    ctxBD.rect(posHS2, ctxBD.canvas.height-24, BDwidth * 2, 25);

    ctxBD.stroke();
}

function textBell(){
    ctxBD.clearRect(2*BDwidth , 0, 64, 64);
    ctxBD.font="bold 16px verdana, sans-serif";
    ctxBD.textAlign="center";
    if(TcurrentSwingDisplayed != null){
        ctxBD.fillStyle = "rgba(240,240,0,1)";
        ctxBD.fillText((TcurrentSwingDisplayed+1).toString(),2*BDwidth + 32,48);
    }
    if ((currentStatus & PAUSED) != 0) {
        ctxBD.fillStyle = "white";
        ctxBD.fillText("Paused",2*BDwidth + 32,32);
        return;        
    }
    if(currentSwingDisplayed != null){
        ctxBD.fillStyle = "white";
        ctxBD.fillText((currentSwingDisplayed+1).toString(),2*BDwidth + 32,32);
    }
}

function clearBell(){
    ctxBD.clearRect(2*BDwidth, 0, 64, 64);
}

function clearStave(){
    ctxAT.clearRect(0, 0, ctxAT.canvas.width, ctxAT.canvas.height);
    ctxATt.clearRect(0, 0, ctxATt.canvas.width, ctxATt.canvas.height);
    var totalHeight = ctxAT.canvas.height-ATbottomMargin-ATtopMargin;
    
    ctxATt.fillStyle="rgba(204,204,230,0.1)";
    ctxATt.fillRect(0,0,ctxATt.canvas.width,ctxATt.canvas.height);
    currentATmargin=0;
    document.getElementById("canvasAT").style.marginLeft = (currentATmargin * -1) + "px";
    
    ctxATt.font = "14px sans serif";
    ctxATt.fillStyle = "rgba(255,150,150,0.7)";
    ctxATt.textAlign = "start";
    ctxATt.textBaseline="top";
    ctxATt.fillText("Slow/Up", 2, ATtopMargin+2);
    ctxATt.textBaseline="bottom";
    ctxATt.fillText("Quick/Down", 2, ATtopMargin+totalHeight-2);

    ctxATt.beginPath();
    ctxATt.strokeStyle = "rgba(255,100,100,0.7)";
//    ctxATt.moveTo(0, ATtopMargin);
//    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin);
    ctxATt.moveTo(0, ATtopMargin+totalHeight*0.0);
    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin+totalHeight*0.0);
    ctxATt.moveTo(0, ATtopMargin+totalHeight*0.25);
    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin+totalHeight*0.25);
    ctxATt.moveTo(0, ATtopMargin+totalHeight*0.75);
    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin+totalHeight*0.75);
    ctxATt.moveTo(0, ATtopMargin+totalHeight*1.0);
    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin+totalHeight*1.0);
//    ctxATt.moveTo(0, ATtopMargin+totalHeight);
//    ctxATt.lineTo(ctxATt.canvas.width, ATtopMargin+totalHeight);
    ctxATt.stroke();

    ctxATt.beginPath();
    ctxATt.strokeStyle = "rgba(192,192,192,0.7)";
    ctxATt.moveTo(0, ATtopMargin+totalHeight*0.5);
    ctxATt.lineTo(ctxAT.canvas.width, ATtopMargin+totalHeight*0.5);
    if(!(currentStatus & PLAYBACK) && !(currentStatus & RECORDINGSESSION)){
        ctxATt.moveTo((ctxATt.canvas.width/2.0)-ATstepWidth,ATtopMargin-15);
        ctxATt.lineTo((ctxATt.canvas.width/2.0)-ATstepWidth,ctxATt.canvas.height);
        ctxATt.moveTo((ctxATt.canvas.width/2.0)+ATstepWidth,ATtopMargin-15);
        ctxATt.lineTo((ctxATt.canvas.width/2.0)+ATstepWidth,ctxATt.canvas.height);
    }
    ctxATt.stroke();
}

function drawBell(angle) {
    var DEG2RAD = Math.PI/180;
    var bx = 2*BDwidth + 32 + (Math.sin((angle - 80) * DEG2RAD) * 10);
    var by = 32 + (Math.cos((angle - 80) * DEG2RAD) * 10);
    var cx = 2*BDwidth + 32 + (Math.sin((angle -  5) * DEG2RAD) * 15);
    var cy = 32 + (Math.cos((angle -  5) * DEG2RAD) * 15);
    var dx = 2*BDwidth + 32 + (Math.sin((angle - 40) * DEG2RAD) * 30);
    var dy = 32 + (Math.cos((angle - 40) * DEG2RAD) * 30);
    var ex = 2*BDwidth + 32 + (Math.sin((angle + 80) * DEG2RAD) * 10);
    var ey = 32 + (Math.cos((angle + 80) * DEG2RAD) * 10);
    var fx = 2*BDwidth + 32 + (Math.sin((angle +  5) * DEG2RAD) * 15);
    var fy = 32 + (Math.cos((angle +  5) * DEG2RAD) * 15);
    var gx = 2*BDwidth + 32 + (Math.sin((angle + 40) * DEG2RAD) * 30);
    var gy = 32 + (Math.cos((angle + 40) * DEG2RAD) * 30);
    ctxBD.clearRect(2*BDwidth, 0, 64, 64);
    ctxBD.beginPath();
    ctxBD.moveTo(2*BDwidth + 32, 32);
    ctxBD.bezierCurveTo(bx, by, cx, cy, dx, dy);
    ctxBD.moveTo(2*BDwidth + 32, 32);
    ctxBD.bezierCurveTo(ex, ey, fx, fy, gx, gy);
    ctxBD.lineWidth = 4;
    ctxBD.strokeStyle = "white";
    ctxBD.stroke();
}

function drawPowerScales(){
    drawTarget();
    var stepSize = (BDwidth * 1.0)/(ROIL-ROIU); // this is how much the position needs to be scaled
    var zero = (0-ROIU) * stepSize;
    var currentStep = null;
    var angleJump = null;

    ctxBDt.font = "12px sans serif";
    ctxBDt.fillStyle = "rgb(255,150,150)";
    ctxBDt.textAlign = "center";
    ctxBDt.textBaseline="bottom";
    ctxBDt.fillText("TDC", posBS2 + BDwidth - zero, ctxBDt.canvas.height);
    ctxBDt.fillText("TDC", posHS1 + zero, ctxBDt.canvas.height);
    ctxBDt.fillText("TDC", posHS2 + BDwidth - zero, ctxBDt.canvas.height);
    ctxBDt.fillText("TDC", posBS1 + zero, ctxBDt.canvas.height);
    ctxBDt.fillText(ROIU.toString(), posHS1, ctxBDt.canvas.height);
    ctxBDt.fillText(ROIU.toString(), posBS1, ctxBDt.canvas.height);
//    ctxBDt.fillText(ROIL.toString(), posBS1, ctxBDt.canvas.height);
//    ctxBDt.fillText(ROIL.toString(), posHS1 + BDwidth, ctxBDt.canvas.height);
//    ctxBDt.fillText(ROIL.toString(), posHS2, ctxBDt.canvas.height);
//    ctxBDt.fillText(ROIL.toString(), posBS1 + BDwidth, ctxBDt.canvas.height);
    ctxBDt.beginPath();
    ctxBDt.strokeStyle = "rgb(255,100,100)";
    ctxBDt.moveTo(posHS1, ctxBDt.canvas.height - 16);
    ctxBDt.lineTo(posHS1, 14);
    ctxBDt.moveTo(posBS1, ctxBDt.canvas.height - 16);
    ctxBDt.lineTo(posBS1, 14);
    
    ctxBDt.moveTo(posBS2 + BDwidth - zero, ctxBDt.canvas.height - 16);
    ctxBDt.lineTo(posBS2 + BDwidth - zero, ctxBDt.canvas.height * 0.7);
    ctxBDt.moveTo(posHS1 + zero, ctxBDt.canvas.height -16);
    ctxBDt.lineTo(posHS1 + zero, ctxBDt.canvas.height * 0.7);
    ctxBDt.moveTo(posHS2 + BDwidth - zero, ctxBDt.canvas.height - 16);
    ctxBDt.lineTo(posHS2 + BDwidth - zero, ctxBDt.canvas.height * 0.7);
    ctxBDt.moveTo(posBS1 + zero, ctxBDt.canvas.height - 16);
    ctxBDt.lineTo(posBS1 + zero, ctxBDt.canvas.height * 0.7);
    ctxBDt.stroke();

    var hiScale = 14;
    var loScale = ((ctxBD.canvas.height - 36)/2) + 14;
    
    ctxBDt.beginPath();
    ctxBDt.textBaseline="top";
    ctxBDt.fillStyle = "rgb(255,150,150)";
    ctxBDt.fillText(scaleValue.toString(), posHS2 + BDwidth +10, hiScale);
    ctxBDt.fillText(scaleValue.toString(), posHS1 + 10, hiScale);
    ctxBDt.fillText((scaleValue/2).toString(), posHS2 + BDwidth +10, loScale);
    ctxBDt.fillText((scaleValue/2).toString(), posHS1 + 10, loScale);
    ctxBDt.stroke();
    
    if (ROIL-ROIU >=60){
        angleJump=20;
        currentStep=20;
    } else {
        angleJump=10;
        currentStep=10;
    }
    ctxBDt.beginPath();
    ctxBDt.strokeStyle = "rgb(255,100,100)";
    ctxBDt.fillStyle = "rgb(255,150,150)";
    ctxBDt.textBaseline="bottom";
    while (currentStep < ROIL){
        ctxBDt.fillText(currentStep.toString(), posBS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height);
        ctxBDt.moveTo(posBS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height - 16);
        ctxBDt.lineTo(posBS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height * 0.7);

        ctxBDt.fillText(currentStep.toString(), posHS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height);
        ctxBDt.moveTo(posHS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height -16);
        ctxBDt.lineTo(posHS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height * 0.7);

        ctxBDt.fillText(currentStep.toString(), posHS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height);
        ctxBDt.moveTo(posHS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height - 16);
        ctxBDt.lineTo(posHS2 + BDwidth - zero - (currentStep * stepSize), ctxBDt.canvas.height * 0.7);

        ctxBDt.fillText(currentStep.toString(), posBS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height);
        ctxBDt.moveTo(posBS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height -16);
        ctxBDt.lineTo(posBS1 + zero + (currentStep * stepSize), ctxBDt.canvas.height * 0.7);

        currentStep+=angleJump;
    }
    ctxBDt.stroke();
    
    
    
    
    ctxBDt.textAlign = "start";
    ctxBDt.textBaseline="alphabetic";

}

function drawTarget(){
    var stepSize = (BDwidth * 1.0)/(ROIL-ROIU);
    var zero = (0-ROIU) * stepSize;

    if (targetAngleHand !== null && targetAngleHand >= ROIU  && targetAngleHand <= ROIL) {
        ctxBDt.beginPath();
        ctxBDt.lineWidth=4;
        ctxBDt.strokeStyle="rgba(0,240,0,0.9)";
        
        ctxBDt.moveTo((BDwidth - zero)-(targetAngleHand * stepSize),14);
        ctxBDt.lineTo((BDwidth - zero)-(targetAngleHand * stepSize), ctxBD.canvas.height-26);

        ctxBDt.moveTo((BDwidth + zero) + (targetAngleHand * stepSize),14);
        ctxBDt.lineTo((BDwidth + zero) + (targetAngleHand * stepSize), ctxBDt.canvas.height-26);
       
        ctxBDt.stroke();
        ctxBDt.lineWidth=1;
    }
    
    if (targetAngleBack !== null && targetAngleBack >= ROIU  && targetAngleBack <= ROIL) {
        ctxBDt.beginPath();
        ctxBDt.lineWidth=4;
        ctxBDt.strokeStyle="rgba(0,240,0,0.9)";
        
        ctxBDt.moveTo(posHS2 + (BDwidth-zero)-(targetAngleBack * stepSize),14);
        ctxBDt.lineTo(posHS2 + (BDwidth-zero)-(targetAngleBack * stepSize), ctxBD.canvas.height-26);

        ctxBDt.moveTo(posHS2 + (BDwidth + zero) + (targetAngleBack * stepSize),14);
        ctxBDt.lineTo(posHS2 + (BDwidth + zero) + (targetAngleBack * stepSize), ctxBDt.canvas.height-26);
       
        ctxBDt.stroke();
        ctxBDt.lineWidth=1;
    }
 }

function preventZoom(e) {
    var t2 = e.timeStamp;
    var t1 = e.currentTarget.dataset.lastTouch || t2;
    var dt = t2 - t1;
    var fingers = e.touches.length;
    e.currentTarget.dataset.lastTouch = t2;
    if (!dt || dt > 500 || fingers > 1) return; // not double-tap
    e.preventDefault();
    e.target.click();
}

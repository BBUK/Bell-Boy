// Copyright (c) 2017 Peter Budd. All rights reserved
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
// bell rope.  The Bell-Boy uses tangential acceleration of the bell as a proxy for force applied.  The
// hardware is currently a Pi Zero running Arch Linux, two MPU6050 breakout boards and the official Pi Wifi dongle
// The MPU6050 breakout boards are mounted on opposite sides of the bell axle such that gravity pulls
// on MPU6050s in the same direction but so that the tangential acceleration applied by the ringer
// works in opposite directions on each sensor.  This makes it easy to separate out tangential acceleration
// due to gravity and tangential acceleration due to force applied.

// I pulled anippets of code from loads of places so if any of you recognise anything you wrote - thanks!

// The script below is the javascript for the front end.  It makes extensive use of HTML5 canvases
// so HTML5 is required.  Websockets are also used to pull data from the Bell-Boy device.


var BGCOLOUR="#686888";

var nonLive=false;
var wsHost="10.0.0.1"

var sampleInterval = 20;  // milliseconds for each sample
var collectChunk = 5; // samples to collect at any one time

var canvasHS1 = document.getElementById("canvasHS1");
var canvasHS2 = document.getElementById("canvasHS2");
var canvasbell = document.getElementById("canvasbell");
var canvasBS1 = document.getElementById("canvasBS1");
var canvasBS2 = document.getElementById("canvasBS2");
var ctxHS1 = canvasHS1.getContext("2d");
var ctxHS2 = canvasHS2.getContext("2d");
var ctxCB = canvasbell.getContext("2d");
var ctxBS1 = canvasBS1.getContext("2d");
var ctxBS2 = canvasBS2.getContext("2d");

var canvasHS1t= document.getElementById("canvasHS1t");
var ctxHS1t = canvasHS1t.getContext("2d");
var canvasHS2t= document.getElementById("canvasHS2t");
var ctxHS2t = canvasHS2t.getContext("2d");
var canvasBS1t= document.getElementById("canvasBS1t");
var ctxBS1t = canvasBS1t.getContext("2d");
var canvasBS2t= document.getElementById("canvasBS2t");
var ctxBS2t = canvasBS2t.getContext("2d");

var strokeTimer=0; // used for time displays under animated bell

var canvasAT=document.getElementById("canvasAT");
var ctxAT=canvasAT.getContext("2d");
//var canvasATBS=document.getElementById("canvasATBS");
//var ctxATBS=canvasATBS.getContext("2d");

var canvasATt=document.getElementById("canvasATt");
var ctxATt=canvasATt.getContext("2d");
//var canvasATBSt=document.getElementById("canvasATBSt");
//var ctxATBSt=canvasATBSt.getContext("2d");
var currentATmargin = 0;
var currentATpixels = 2;
var ATbottomMargin=20; // pixels at bottom of AT canvas for indexes

var oldBellAngle = 0;
var wsOpened = false;

var scaleValue = 0.0;

var ROIRanges = [[-30,90],[-20,70],[-10,60],[-10,50],[-10,40],[-10,30]];
var currentROI=3;
var ROIU = ROIRanges[currentROI][0];
var ROIL = ROIRanges[currentROI][1];

//formula for calculation [2]*[1] = [0]*collectChunk
//var playbackRanges = [[150,150,10],[100,100,10],[80,80,10],[50,100,5],[30,75,4],[10,100,1]]; // percent to report, slowdown percentage, number of iterations per cycle
var playbackRanges = [[200,200,5],[150,150,5],[100,100,5],[80,80,5],[50,62.5,4],[30,75,2],[10,50,1]]; // percent to report, slowdown percentage, number of iterations per cycle

var currentPlaybackSpeed = 2;
var targets = [ "none", "-10", "-5", "-7", "-2" , "0", "2", "5", "7" , "10" ];
var scales = [ 10000, 8000, 5000, 3000, 2000, 1000, 700, 500 ];

var currentPlaybackPosition = 1;
var playintervalID = null;
var statusintervalID = null;
var liveintervalID = null;

var currentStatus = 0;
var DOWNLOADINGFILE=1;
var RECORDINGSESSION=2;
var SESSIONLOADED=4;
var FAVOURITEDISPLAYED=8;
var PLAYBACK=16;
var LASTHS1=32;
var LASTHS2=64;
var LASTBS1=128;
var LASTBS2=256;
var HELPDISPLAYED=512;
var PAUSED=1024;
var GRIDLINESDISPLAYED=2048;
var LIVEVIEW=4096;
var ABORTFLAG = 8192;

var targetAngleHand=null;
var targetAngleBack=null;

var currentSwingDisplayed=null;
var sample = [];
var swingStarts = [];
var halfSwingStarts=[];
var template = [];

ws = new WebSocket("ws://" + wsHost + "/bell_boy");

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
    option.text=ROIRanges[i][0].toString() + " to " + ROIRanges[i][1].toString()
    document.getElementById("zoomSelect").add(option);
}
document.getElementById("zoomSelect").selectedIndex = currentROI.toString();

document.getElementById("speedSelect").options.length = 0;
for (var i=0; i < playbackRanges.length; i++) {
    var option = document.createElement("option");
    option.text=playbackRanges[i][0].toString() + "%"
    document.getElementById("speedSelect").add(option);
}
document.getElementById("speedSelect").selectedIndex = currentPlaybackSpeed;

document.getElementById("gridSelect").options.length = 0;
var option = document.createElement("option");
option.text="yes"
document.getElementById("gridSelect").add(option);
option = document.createElement("option");
option.text="no"
document.getElementById("gridSelect").add(option);
document.getElementById("gridSelect").selectedIndex = 0;

document.getElementById("scaleSelect").options.length = 0;
for (var i=0; i < scales.length; i++) {
    var option = document.createElement("option");
    option.text=scales[i].toString()
    document.getElementById("scaleSelect").add(option);
}
document.getElementById("scaleSelect").selectedIndex = 2;  // start at 2000
scaleValue=parseFloat(scales[document.getElementById("scaleSelect").selectedIndex])

////////////////////////////////////////////////////////////////
//                 WEBSOCKET HANDLER FUNCTIONS                   //
////////////////////////////////////////////////////////////////

ws.onopen = function(){
    wsOpened=true;
    setStatus("Link open");
    document.getElementById("openSelect").options.length = 0;
    ws.send("FILE:");
};

ws.onmessage = function (event) {
    var dataBack = event.data;
    if (dataBack.slice(0,5) == "FILE:"){
        var option = document.createElement("option");
        option.text=dataBack.slice(5);
        document.getElementById("openSelect").add(option);
        return;
    }
    if (dataBack.slice(0,5) == "DATA:"){
        var sampArray = dataBack.split("DATA:");
        var arrayLength = sampArray.length;
        var added = 0;
        for (var i = 0; i < arrayLength; i++) {
            if (sampArray[i].length > 0) {   // have more checks for good data
                sample[sample.length] = extractData(sampArray[i]);
                added+=1;
            }
        }
        setStatus("Loaded: " + added.toString() + " samples. Now Have: " + sample.length.toString());
        return;
    }
    if (dataBack.slice(0,5) == "LIVE:"){
        if ((currentStatus & LIVEVIEW) == 0 && (currentStatus & RECORDINGSESSION) == 0) { return; }  // server may push data after stopping
        var sampArray = dataBack.split("LIVE:");
        var arrayLength = sampArray.length;
        for (var i = 0; i < arrayLength; i++) {
            if (sampArray[i].length > 0) {   // have more checks for good data
                sample[sample.length] = extractData(sampArray[i]);
            }
        }
        return;
    }

    if ((dataBack.slice(0,5) == "LFIN:") || (dataBack.slice(0,5) == "STPD:")) {
        if ((currentStatus & DOWNLOADINGFILE) !=0){
            currentStatus &= ~DOWNLOADINGFILE;
        } else if ((currentStatus & LIVEVIEW) !=0) {
            currentStatus &= ~LIVEVIEW;
        } else if ((currentStatus & RECORDINGSESSION) !=0) {
            currentStatus &= ~RECORDINGSESSION;
        }
        if ((currentStatus & ABORTFLAG) !=0) {
            currentStatus &= ~ABORTFLAG;
            sample = [];
            swingStarts=[];
            currentPlaybackPosition = 1;
            currentSwingDisplayed = null;
            return;
        }
        currentStatus |= SESSIONLOADED;
        updateIcons();
        swingStarts=[];
        halfSwingStarts=[];
//        swingStarts[0] = 3; // ditch first couple of samples - could be 1 but meh
        currentPlaybackPosition = 1;
        currentSwingDisplayed = null;
        var i = null, j=null , k=null;
        var arrayLength=sample.length;

//        while (i < arrayLength && sample[i][0] < 180) i++;
//        if (i == arrayLength) {
//            setStatus("Loaded: " + sample.length.toString() + " samples. Should Have: " + dataBack.slice(5) + " No strokes found");
//            return;
//        }

        for (j = 0; j < arrayLength; j++){
            if (sample[j][0] > 50 && sample[j][0] < 70 && sample[j][1] > 0) { // this is point that bell is moving heathily down @ handstroke
                k = j;
                if (swingStarts.length != 0) {
                    i = swingStarts[swingStarts.length-1]+1
                } else {
                    i=0;
                }
                while (k > i) {
                    k--; // step back from healthily down point
                    if (sample[k][1] <= 0.05){ // find point when bell is nearly stationary and define that as start of stroke
                        swingStarts[swingStarts.length] = k+1;
                        break;
                    }
                }
                if (k == i) {
                    setStatus("Loaded: " + sample.length.toString() + " samples. Should Have: " + dataBack.slice(5) + " Funny swing found. Possible incorrect data. But " + swingStarts.length.toString() + " strokes found. " + swingStarts.toString());
                    return;
                }
                while (j < arrayLength && sample[j][0] < 180) j++;  // move forward through sample past BDC
                if (j == arrayLength ) break;
            }
            if (sample[j][0] > 290 && sample[j][0] < 310 && sample[j][1] < 0) { // point where bell moving healthily down @ backstroke
                k = j;
                if (halfSwingStarts.length != 0) {
                    i = halfSwingStarts[halfSwingStarts.length-1]+1
                } else {
                    i=0;
                }
                while (k > i) { // step back from healthily down point
                    k--;
                    if (sample[k][1] >= -0.05){ // find point when bell is nearly stationary and define that as start of half stroke
                        halfSwingStarts[halfSwingStarts.length] = k+1;
                        break;
                    }
                }
                if (k == i) {
                    setStatus("Loaded: " + sample.length.toString() + " samples. Should Have: " + dataBack.slice(5) + " Funny half swing found. Possible incorrect data. But " + swingStarts.length.toString() + " strokes found. " + swingStarts.toString());
                    return;
                }
                while (j < arrayLength && sample[j][0] > 180) j++;
            }
        }
        setStatus("Loaded: " + sample.length.toString() + " samples. Should Have: " + dataBack.slice(5) + ". " + swingStarts.length.toString() + " strokes found.");
//        setStatus("Loaded: " + halfSwingStarts);

        // rebuild file list
        document.getElementById("openSelect").options.length = 0;
        ws.send("FILE:");
        return;
    }
    if (dataBack.slice(0,5) == "STRT:"){
        setStatus("Recording started");
        return;
    }
    if (dataBack.slice(0,5) == "EIMU:"){
        setStatus("IMU not active.  Is one connected to device???");
        return;
    }
    if (dataBack.slice(0,5) == "ESTD:"){
        setStatus("Bell not at stand.  Aborting");
        if ((currentStatus & RECORDINGSESSION) != 0) {
            document.getElementById("recordIcon").onclick()
        } else if ((currentStatus & LIVEVIEW) != 0) {
             document.getElementById("liveIcon").onclick()
        }
        currentStatus |= ABORTFLAG;
        return;
    }
    if (dataBack.slice(0,5) == "EMOV:"){
        setStatus("Bell moving.  Aborting");
        if ((currentStatus & RECORDINGSESSION) != 0) {
            document.getElementById("recordIcon").onclick()
        } else if ((currentStatus & LIVEVIEW) != 0) {
             document.getElementById("liveIcon").onclick()
        }
        currentStatus |= ABORTFLAG;
        return;
    }

    setStatus(dataBack);
};

ws.onclose = function(event){
    wsOpened=false;
    setStatus("Error. Resetting device. Please press refresh.");
    if ((currentStatus & RECORDINGSESSION) != 0  || (currentStatus & LIVEVIEW) != 0) {
        currentStatus &= ~RECORDINGSESSION;
        currentStatus &= ~LIVEVIEW;
        updateIcons();
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        clearBell();
        currentPlaybackPosition=1;
    }
};

////////////////////////////////////////////////////////////////
//                       DATA FUNCTIONS                          //
////////////////////////////////////////////////////////////////

function extractData(datastring){
    var entries = datastring.split(",");
//    return [parseFloat(entries[0].slice(2)), parseFloat(entries[1].slice(2)), parseFloat(entries[2].slice(2)) * 4000.0];
    return [parseFloat(entries[0].slice(2)), parseFloat(entries[1].slice(2)), parseFloat(entries[2].slice(2))];
//    return [parseFloat(entries[0].slice(2)), parseFloat(entries[1].slice(2)), parseFloat(entries[2].slice(2))+10000*Math.sin(parseFloat(entries[0].slice(2))*3.142/180)];

}

function playbackSample(){
    var iterations = playbackRanges[currentPlaybackSpeed][2]; // do this so that slower speeds are smoother
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

function showLive(){
    ws.send("LDAT:");
    var iterations = (sample.length-1) - currentPlaybackPosition;
    if (iterations < 5) return; // draw not fewer than 5 samples at a time
    drawSamples(currentPlaybackPosition,iterations);
    currentPlaybackPosition += iterations;
    setStatus("Loaded: " + currentPlaybackPosition.toString());
}

function drawTimer(position, templated){
    var timeElapsed = ((position-strokeTimer)*sampleInterval)/1000.0;
    if (timeElapsed < 0.2) return; // assume a glitch
    ctxCB.font = "12px sans serif";
    ctxCB.fillStyle = "white";
    if ((currentStatus & LASTBS2) != 0 ) {  // i.e. timing backstroke
        ctxCB.clearRect(0, ctxCB.canvas.height-20, ctxCB.canvas.width, 10);
        if (timeElapsed > 0.5) {
            ctxCB.textAlign = "right";
            ctxCB.fillText(timeElapsed.toFixed(2)+"s >", ctxCB.canvas.width, ctxCB.canvas.height-10);
        }
    } else {
        ctxCB.clearRect(0, ctxCB.canvas.height-50, ctxCB.canvas.width, 10);
        if (timeElapsed > 0.5) {
            ctxCB.textAlign = "left";
            ctxCB.fillText("< " + timeElapsed.toFixed(2)+"s", 0, ctxCB.canvas.height-40);
        }
    }
    strokeTimer=position;
}

function drawSamples(position,iterations){
    var maxlength = 0.75*radius;
    for (var i = 0; i < iterations; i++){
        var dataEntryOld = sample[position + i -1].slice();
        var dataEntryCurrent = sample[position + i].slice();
        var dataEntryNext = sample[position + i + 1].slice();
        drawBell(180-dataEntryCurrent[0]);
        drawAT(dataEntryCurrent[2]);
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
                drawAccel(ctxBS2,dataEntryOld[0],ROIU,maxlength,true);
                drawAccel(ctxHS1,ROIU,startAngle,maxlength,true);
                if ((currentStatus & LASTBS2) != 0 ) {
                    drawTimer(position + i, false);
                } else {
                    strokeTimer = 0; // not started yet
                }
                currentStatus &= ~(LASTBS2 | LASTBS1 | LASTHS2);
                currentStatus |= LASTHS1;
            }
            drawAccel(ctxHS1,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)),false);
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] >= 0) { // within ROI for HS2
            var endAngle = Math.min((360-ROIU),dataEntryCurrent[0]);
            var startAngle = Math.max(dataEntryOld[0], (360-ROIL));
            if (startAngle >= endAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTHS2) == 0 ) { // set flags if we have not been before this swing
                currentStatus &= ~(LASTHS1 | LASTBS1 | LASTBS2);
                currentStatus |= LASTHS2;
            }
            drawAccel(ctxHS2,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)),false);
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] < 0) { // within ROI for BS1

            if (dataEntryNext[0] <= (360 - ROIL)) {
                dataEntryCurrent[0] = (360 - ROIL); // tidy up at end of ROI
            }
            var startAngle = Math.min((360-ROIU),dataEntryOld[0]);
            var endAngle = dataEntryCurrent[0];
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTBS1) == 0 ) { // clear highest part of this and previous ROI if this is the first time
                drawAccel(ctxHS2,dataEntryOld[0],(360-ROIU),maxlength,true);
                drawAccel(ctxBS1,(360-ROIU),startAngle,maxlength,true);
                if ((currentStatus & LASTHS2) != 0 ) {
                    drawTimer(position + i, false);
                }
                currentStatus &= ~(LASTBS2 | LASTHS1 | LASTHS2);
                currentStatus |= LASTBS1;
            }
            drawAccel(ctxBS1,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)),false);

        } else if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] < 0) { // within ROI for BS2
            var endAngle = Math.max(ROIU,dataEntryCurrent[0]);
            var startAngle = Math.min(dataEntryOld[0],ROIL);
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            if ((currentStatus & LASTBS2) == 0 ) { // set flags if we have not been before this swing
                currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2);
                currentStatus |= LASTBS2;
            }
            drawAccel(ctxBS2,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)),false);
        }
    }
}

function drawAT(accn){
    var halfHeight = (ctxAT.canvas.height-ATbottomMargin)/2;
    var ypos = halfHeight + (halfHeight*(accn/scaleValue)/2);
    if (ypos < 0) ypos=0;
    if (ypos > halfHeight *2) ypos = halfHeight*2;
    if (currentATmargin >= ctxAT.canvas.width/2) currentATmargin = 0;
    currentATmargin += currentATpixels;
    ctxAT.clearRect(currentATmargin-currentATpixels,0,currentATpixels,ctxAT.canvas.height);
    ctxAT.clearRect(currentATmargin-currentATpixels+ctxAT.canvas.width/2,0,currentATpixels,ctxAT.canvas.height);
    ctxAT.beginPath();
    ctxAT.moveTo(currentATmargin-currentATpixels,ypos);
    ctxAT.lineTo(currentATmargin,ypos);
    ctxAT.moveTo(currentATmargin-currentATpixels+ctxAT.canvas.width/2,ypos);
    ctxAT.lineTo(currentATmargin+ctxAT.canvas.width/2,ypos);
    if ((currentStatus & LASTHS1) != 0 ||
        (currentStatus & LASTHS2) != 0) {
        ctxAT.strokeStyle="#FF8080";
    } else {
        ctxAT.strokeStyle="#80FF80";
    }
    ctxAT.lineWidth=2;
    ctxAT.stroke();
    document.getElementById("canvasAT").style.marginLeft = (currentATmargin * -1) + "px";
}

function clearTemplates(){
    ctxHS1t.save();
    ctxHS1t.setTransform(1, 0, 0, 1, 0, 0);
    ctxHS1t.clearRect(0, 0, ctxHS1t.canvas.width, ctxHS1t.canvas.height);
    ctxHS1t.restore();

    ctxHS2t.save();
    ctxHS2t.setTransform(1, 0, 0, 1, 0, 0);
    ctxHS2t.clearRect(0, 0, ctxHS2t.canvas.width, ctxHS2t.canvas.height);
    ctxHS2t.restore();

    ctxBS1t.save();
    ctxBS1t.setTransform(1, 0, 0, 1, 0, 0);
    ctxBS1t.clearRect(0, 0, ctxBS1t.canvas.width, ctxBS1t.canvas.height);
    ctxBS1t.restore();

    ctxBS2t.save();
    ctxBS2t.setTransform(1, 0, 0, 1, 0, 0);
    ctxBS2t.clearRect(0, 0, ctxBS2t.canvas.width, ctxBS2t.canvas.height);
    ctxBS2t.restore();

}

function drawSamplesOnTemplate(){
    clearTemplates();
    var maxlength = 0.75*radius;
    for (var i = 1; i < template.length-1; i++){
        var dataEntryOld = template[i -1].slice();
        var dataEntryCurrent = template[i].slice();
        var dataEntryNext = template[i + 1].slice();
        if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] >= 0) { // within ROI for HS1

            if (dataEntryNext[0] >= ROIL) {
                dataEntryCurrent[0] = ROIL; // tidy up at end of ROI
            }
            var startAngle = Math.max(ROIU,dataEntryOld[0]);
            var endAngle = dataEntryCurrent[0];
            if (endAngle <= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            drawAccelT(ctxHS1t,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)));
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] >= 0) { // within ROI for HS2
            var endAngle = Math.min((360-ROIU),dataEntryCurrent[0]);
            var startAngle = Math.max(dataEntryOld[0], (360-ROIL));
            if (startAngle >= endAngle) {
                continue; // for moment don't swap, just do nothing
            }
            drawAccelT(ctxHS2t,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)));
        } else if (dataEntryCurrent[0] > (360 - ROIL) && dataEntryCurrent[0] < (360 - ROIU) && dataEntryCurrent[1] < 0) { // within ROI for BS1

            if (dataEntryNext[0] <= (360 - ROIL)) {
                dataEntryCurrent[0] = (360 - ROIL); // tidy up at end of ROI
            }
            var startAngle = Math.min((360-ROIU),dataEntryOld[0]);
            var endAngle = dataEntryCurrent[0];
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            drawAccelT(ctxBS1t,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)));

        } else if (dataEntryCurrent[0] > ROIU && dataEntryCurrent[0] < ROIL && dataEntryCurrent[1] < 0) { // within ROI for BS2
            var endAngle = Math.max(ROIU,dataEntryCurrent[0]);
            var startAngle = Math.min(dataEntryOld[0],ROIL);
            if (endAngle >= startAngle) {
                continue; // for moment don't swap, just do nothing
            }
            drawAccelT(ctxBS2t,startAngle,endAngle,maxlength*(Math.min(Math.abs(dataEntryCurrent[2])/scaleValue,1)));
        }
    }
    drawTDCs();  // function checks what needs to be drawn on template like gridlines and target angles
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
//                    ICON CLICK FUNCTIONS                       //
////////////////////////////////////////////////////////////////

fileOpenButton.onclick = function() {
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    var e = document.getElementById("openSelect");
    sample = [];
    ws.send("LOAD:" + e.options[e.selectedIndex].text);
    openModal.style.display = "none";
    currentStatus |= DOWNLOADINGFILE;
    currentStatus &= ~SESSIONLOADED;
    currentSwingDisplayed = null;
    updateIcons();
}

targetSelectHand.onchange = function() {
    var h = document.getElementById("targetSelectHand");
    var t = h.options[h.selectedIndex].text
    if (t.isNaN) {
        targetAngleHand=null
    } else {
        targetAngleHand=parseInt(t)
    }
    if ((currentStatus & FAVOURITEDISPLAYED) != 0){
        drawSamplesOnTemplate();
    } else {
        clearTemplates();
        drawTDCs();
    }

}

targetSelectBack.onchange = function() {
    var b = document.getElementById("targetSelectBack");
    var t = b.options[b.selectedIndex].text
    if (t.isNaN) {
        targetAngleBack=null
    } else {
        targetAngleBack=parseInt(t)
    }
    if ((currentStatus & FAVOURITEDISPLAYED) != 0){
        drawSamplesOnTemplate();
    } else {
        clearTemplates();
        drawTDCs();
    }
}

zoomSelect.onchange = function(){
    currentROI = document.getElementById("zoomSelect").selectedIndex;
    ROIU = ROIRanges[currentROI][0];
    ROIL = ROIRanges[currentROI][1];
    drawFrame(ctxHS1);
    drawFrame(ctxHS2);
    drawFrame(ctxBS1);
    drawFrame(ctxBS2);
    if ((currentStatus & FAVOURITEDISPLAYED) != 0) {
        drawSamplesOnTemplate();
    } else {
        clearTemplates();
        drawTDCs();
    }
    if (currentSwingDisplayed == null) return;

    if (currentSwingDisplayed == swingStarts.length - 1){
        iterations = (sample.length - 3) - swingStarts[currentSwingDisplayed];
    } else {
        iterations = (swingStarts[currentSwingDisplayed +1]-1) - swingStarts[currentSwingDisplayed];
    }
    drawSamples(swingStarts[currentSwingDisplayed],iterations);
    textBell((currentSwingDisplayed+1).toString());
    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
}

speedSelect.onchange = function(){
    currentPlaybackSpeed = document.getElementById("speedSelect").selectedIndex;
// consider whether to allow this to be changed during active playback
    if (playintervalID != null) {
        clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,sampleInterval*collectChunk*(100/playbackRanges[currentPlaybackSpeed][1]));
    }
}

/*
plusIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if (currentPlaybackSpeed == 0) return;
    currentPlaybackSpeed -= 1;
    if (playintervalID != null) {
        clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,200*(100/playbackRanges[currentPlaybackSpeed][1]));
    }
    setStatus("Playback speed set to " + playbackRanges[currentPlaybackSpeed][0].toString() + "%");
    updateIcons();
}

minusIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if (currentPlaybackSpeed >= playbackRanges.length -1) return;
    currentPlaybackSpeed += 1;
    if (playintervalID != null) {
        clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,200*(100/playbackRanges[currentPlaybackSpeed][1]));
    }
    setStatus("Playback speed set to " + playbackRanges[currentPlaybackSpeed][0].toString() + "%");
    updateIcons();
}
*/


gridSelect.onchange = function() {
    if (document.getElementById("gridSelect").selectedIndex == 0){ //yes
        currentStatus |= GRIDLINESDISPLAYED;
        if ((currentStatus & FAVOURITEDISPLAYED) != 0){
            drawSamplesOnTemplate();
        } else {
            drawTDCs();
        }

    } else { // no
        currentStatus &= ~GRIDLINESDISPLAYED;
        if ((currentStatus & FAVOURITEDISPLAYED) != 0){
            drawSamplesOnTemplate();
        } else {
            clearTemplates();
            drawTDCs(); // draw target if necessary
        }
    }
}

/*
gridlinesIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & GRIDLINESDISPLAYED) == 0) {
        currentStatus |= GRIDLINESDISPLAYED;
        if ((currentStatus & FAVOURITEDISPLAYED) != 0){
            drawSamplesOnTemplate();
        } else {
            drawTDCs();
        }
    } else {
        currentStatus &= ~GRIDLINESDISPLAYED;
        if ((currentStatus & FAVOURITEDISPLAYED) != 0){
            drawSamplesOnTemplate();
        } else {
            clearTemplates();
            drawTDCs(); // draw target if necessary
        }
    }
    updateIcons();
}
*/


scaleSelect.onchange = function() {
    scaleValue=parseFloat(scales[document.getElementById("scaleSelect").selectedIndex])
    drawFrame(ctxHS1);
    drawFrame(ctxHS2);
    drawFrame(ctxBS1);
    drawFrame(ctxBS2);
    clearAT();
    if ((currentStatus & FAVOURITEDISPLAYED) != 0) {
        drawSamplesOnTemplate();
    } else {
        clearTemplates();
        drawTDCs();
    }
    if (currentSwingDisplayed == null) return;

    if (currentSwingDisplayed == swingStarts.length - 1){
        iterations = (sample.length - 3) - swingStarts[currentSwingDisplayed];
    } else {
        iterations = (swingStarts[currentSwingDisplayed +1]-1) - swingStarts[currentSwingDisplayed];
    }
    drawSamples(swingStarts[currentSwingDisplayed],iterations);
    textBell((currentSwingDisplayed+1).toString());
    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
}


recordIcon.onclick=function(){
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & LIVEVIEW) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) {
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
}

recordButton.onclick = function() {
    var fileName = document.getElementById("recordFileName").value;
    patt = new RegExp("/^[a-z0-9_. ()-]+$/i");
    if (patt.test(fileName)) {
        recordModal.style.display = "none";
        currentStatus |= RECORDINGSESSION;
        currentStatus &= ~SESSIONLOADED;
        updateIcons();
        drawFrame(ctxHS1);
        drawFrame(ctxHS2);
        drawFrame(ctxBS1);
        drawFrame(ctxBS2);
        sample = [];
        clearAT();
        currentPlaybackPosition = 1; // needs to be one as we are doing comparison with previous entry removed so that playback starts from last swing displayed
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=setInterval(showLive,sampleInterval*collectChunk);
        currentSwingDisplayed=null;
        ws.send("STRT:" + fileName);
    } else {
        document.getElementById("nameInvalid").style.visibility = "visible";
    }
};

liveIcon.onclick = function() {
    if (nonLive){
        alert("Not implemented for this demo");
        return;
    }
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & LIVEVIEW) != 0) {
        currentStatus &= ~LIVEVIEW;
        updateIcons();
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=null;
        currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
        clearBell();
        currentPlaybackPosition=1;
        ws.send("STOP:");
    } else {
        currentStatus |= LIVEVIEW;
        currentStatus &= ~SESSIONLOADED;
        updateIcons();
        drawFrame(ctxHS1);
        drawFrame(ctxHS2);
        drawFrame(ctxBS1);
        drawFrame(ctxBS2);
        clearAT();
        sample = [];
        currentPlaybackPosition = 1; // needs to be one as we are doing comparison with previous entry removed so that playback starts from last swing displayed
        if (liveintervalID != null) clearInterval(liveintervalID);
        liveintervalID=setInterval(showLive,sampleInterval*collectChunk); // 200ms
        currentSwingDisplayed=null;
        ws.send("STRT:");
    }
};
// to do, when PLAYED THEN stopped scaling does not work
// set templte displayed so we know to scale template

powerIcon.onclick = function() {
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if (confirm("Are you sure you want to power off?  Please press Cancel for this demo.")) ws.send("SHDN:");
    return;
};

playIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;

    if ((currentStatus & PLAYBACK) != 0) {
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
        drawFrame(ctxHS1);
        drawFrame(ctxHS2);
        drawFrame(ctxBS1);
        drawFrame(ctxBS2);
        clearAT();
//        currentPlaybackPosition =1; // needs to be one as we are doing comparison with previous entry removed so that playback starts from last swing displayed
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,sampleInterval*collectChunk*(100/playbackRanges[currentPlaybackSpeed][1])); // 100ms = 10 samples per iteration
        currentSwingDisplayed=null;
    }
};

pauseIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & PLAYBACK) == 0) return;

    if ((currentStatus & PAUSED) != 0) {
        currentStatus &= ~PAUSED
        if (playintervalID != null) clearInterval(playintervalID);
        playintervalID=setInterval(playbackSample,sampleInterval*collectChunk*(100/playbackRanges[currentPlaybackSpeed][1]));
        clearBell();
    } else {
        currentStatus |= PAUSED;
        if (playintervalID != null) clearInterval(playintervalID);
        textBell("Paused");
    }
};

backIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if (currentSwingDisplayed == 0) return;
    if (currentSwingDisplayed == null) currentSwingDisplayed=1;
    currentSwingDisplayed -= 1;
    drawFrame(ctxHS1);
    drawFrame(ctxHS2);
    drawFrame(ctxBS1);
    drawFrame(ctxBS2);
    drawSamples(swingStarts[currentSwingDisplayed],(swingStarts[currentSwingDisplayed+1]-1) - swingStarts[currentSwingDisplayed]);
    textBell((currentSwingDisplayed+1).toString());
    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
    updateIcons();
};

forwardIcon.addEventListener("touchstart", preventZoom);
forwardIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if (currentSwingDisplayed == null) currentSwingDisplayed = -1;
    if (currentSwingDisplayed == swingStarts.length - 1) return;
    var iterations = null;
    currentSwingDisplayed += 1;
    if (currentSwingDisplayed == swingStarts.length - 1){
        iterations = (sample.length - 3) - swingStarts[currentSwingDisplayed];
    } else {
        iterations = (swingStarts[currentSwingDisplayed +1]-1) - swingStarts[currentSwingDisplayed];
    }
    drawFrame(ctxHS1);
    drawFrame(ctxHS2);
    drawFrame(ctxBS1);
    drawFrame(ctxBS2);
    drawSamples(swingStarts[currentSwingDisplayed],iterations);
    textBell((currentSwingDisplayed+1).toString());
    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
    updateIcons();
};

favIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if (currentSwingDisplayed == null) return;

    if (currentSwingDisplayed == swingStarts.length - 1){
        iterations = (sample.length - 3) - swingStarts[currentSwingDisplayed];
    } else {
        iterations = (swingStarts[currentSwingDisplayed +1]-1) - swingStarts[currentSwingDisplayed];
    }
    template=[];
    for (var i = 0; i<iterations; i++) template[template.length]=sample[swingStarts[currentSwingDisplayed]+i].slice();

    drawSamplesOnTemplate();
//    currentStatus &= ~(LASTHS1 | LASTBS1 | LASTHS2 | LASTBS2);
    currentStatus |= FAVOURITEDISPLAYED;
    updateIcons();
};

unstarIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    if ((currentStatus & SESSIONLOADED) == 0) return;
    if ((currentStatus & HELPDISPLAYED) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;

    template=[];
    clearTemplates();
    currentStatus &= ~FAVOURITEDISPLAYED;
    drawTDCs();
    updateIcons();
};

helpIcon.onclick=function(){
    if ((currentStatus & DOWNLOADINGFILE) != 0) return;
    if ((currentStatus & PLAYBACK) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;

    if ((currentStatus & HELPDISPLAYED) != 0) {
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
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & PLAYBACK) != 0 ||
        (currentStatus & LIVEVIEW) != 0 ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("recordIcon").src = "circle-record-inactive.png";
    } else {
        if ((currentStatus & RECORDINGSESSION) != 0) {
            document.getElementById("recordIcon").src = "circle-stop-active.png";
        } else {
            document.getElementById("recordIcon").src = "circle-record-active.png";
        }
    }

    // powerIcon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & PLAYBACK) != 0 ||
        (currentStatus & LIVEVIEW) != 0 ||
        (currentStatus & RECORDINGSESSION) != 0) {
        document.getElementById("powerIcon").src = "circle-power-inactive.png";
    } else {
        document.getElementById("powerIcon").src = "circle-power-active.png";
    }

    // settings icon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("settingsIcon").src = "settings-inactive.png";
    } else {
        document.getElementById("settingsIcon").src = "settings-active.png";
    }


    //liveIcon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & PLAYBACK) != 0 ||
        (currentStatus & RECORDINGSESSION) != 0 ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("liveIcon").src = "live-inactive.png";
    } else {
        if ((currentStatus & LIVEVIEW) != 0) {
            document.getElementById("liveIcon").src = "live-stop.png";
        } else {
            document.getElementById("liveIcon").src = "live-active.png";
        }

    }

    //downloadIcon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & PLAYBACK) != 0 ||
        (currentStatus & LIVEVIEW) != 0 ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("downloadIcon").src = "cloud-download-inactive.png";
    } else {
        document.getElementById("downloadIcon").src = "cloud-download-active.png";
    }

    //playIcon
    if ((currentStatus & SESSIONLOADED) == 0) {
        document.getElementById("playIcon").src = "circle-play-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) != 0 ||
            (currentStatus & LIVEVIEW) != 0 ||
            (currentStatus & HELPDISPLAYED) != 0) {
            document.getElementById("playIcon").src = "circle-play-inactive.png";
        } else {
            if ((currentStatus & PLAYBACK) != 0) {
                document.getElementById("playIcon").src = "circle-stop-active.png";
            } else {
                document.getElementById("playIcon").src = "circle-play-active.png";
            }
        }
    }

    //pauseIcon
    if ((currentStatus & PLAYBACK) == 0) {
        document.getElementById("pauseIcon").src = "circle-pause-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) != 0 ||
            (currentStatus & LIVEVIEW) != 0 ||
            (currentStatus & HELPDISPLAYED) != 0) {
            document.getElementById("pauseIcon").src = "circle-pause-inactive.png";
        } else {
                document.getElementById("pauseIcon").src = "circle-pause-active.png";
        }
    }

    //skipIcons and fav icons
    if ((currentStatus & SESSIONLOADED) == 0) {
        document.getElementById("backIcon").src = "circle-back-inactive.png";
        document.getElementById("forwardIcon").src = "circle-forward-inactive.png";
        document.getElementById("favIcon").src = "circle-heart-inactive.png";
    } else {
        if ((currentStatus & DOWNLOADINGFILE) != 0 ||
            (currentStatus & PLAYBACK) != 0 ||
            (currentStatus & LIVEVIEW) != 0 ||
            (currentStatus & HELPDISPLAYED) != 0) {
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

    // helpIcon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & LIVEVIEW) != 0 ||
        (currentStatus & RECORDINGSESSION) != 0 ||
        (currentStatus & PLAYBACK) != 0) {
        document.getElementById("helpIcon").src = "help-inactive.png";
    } else {
        if ((currentStatus & HELPDISPLAYED) != 0) {
            document.getElementById("helpIcon").src = "door-out.png";
        } else {
            document.getElementById("helpIcon").src = "help.png";
        }
    }

    //remove template icon
    if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        ((currentStatus & PLAYBACK) != 0 && (currentStatus & PAUSED) == 0) ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("unstarIcon").src = "circle-cross-inactive.png";

    } else {
        if ((currentStatus & FAVOURITEDISPLAYED) != 0){
            document.getElementById("unstarIcon").src = "circle-cross-active.png";
        } else {
            document.getElementById("unstarIcon").src = "circle-cross-inactive.png";
        }
    }

    // settingsicon
       if ((currentStatus & DOWNLOADINGFILE) != 0 ||
        (currentStatus & PLAYBACK) != 0 ||
        (currentStatus & LIVEVIEW) != 0 ||
        (currentStatus & RECORDINGSESSION) != 0 ||
        (currentStatus & HELPDISPLAYED) != 0) {
        document.getElementById("settingsIcon").src = "settings-inactive.png";
    } else {
        document.getElementById("settingsIcon").src = "settings-active.png";
    }


}
////////////////////////////////////////////////////////////////
//                       MODAL FUNCTIONS                         //
////////////////////////////////////////////////////////////////


var openModal = document.getElementById("openModal");
var openBtn = document.getElementById("downloadIcon");
var openSpan = document.getElementsByClassName("close")[0];

var recordModal = document.getElementById("recordModal");
var recordSpan = document.getElementsByClassName("close")[1];


var settingsModal = document.getElementById("settingsModal");
var settingsBtn = document.getElementById("settingsIcon");
var settingsSpan = document.getElementsByClassName("close")[2];

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
    if ((currentStatus & LIVEVIEW) != 0) return;
    if ((currentStatus & RECORDINGSESSION) != 0) return;
    settingsModal.style.display = "block";
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
};

////////////////////////////////////////////////////////////////
//                   WINDOW RESIZE FUNCTIONS                     //
////////////////////////////////////////////////////////////////


//addEvent(window, "resize", recalculateSize());
window.addEventListener("resize", function(event){
  recalculateSize();
  if ((currentStatus & FAVOURITEDISPLAYED) !=0) {
      drawSamplesOnTemplate();
  } else {
    drawTDCs();
  }
});
window.addEventListener("load", function(event){
    recalculateSize();
    updateIcons();
    currentStatus |= GRIDLINESDISPLAYED;  // have these displayed by default
    drawTDCs();
    clearAT();
    document.body.addEventListener("touchstart", preventZoom); // doesn't appear to work
});

function recalculateSize() {
    var winWidth =  window.innerWidth;
    var rightWidth = Math.max(800,winWidth-16);

    var canvasWidth=((rightWidth-12)/4)-21;
    var canvasHeight=(((rightWidth-21)/4)-21)/1.5;

    var winHeight =  window.innerHeight;
    var rightHeight = Math.max (((((rightWidth-21)/4)-19)/1.5),winHeight - 16 - 172 - 25);

    document.getElementById("bellGraphics").style.width = rightWidth + "px";
    document.getElementById("bellGraphics").style.height = rightHeight + "px";
    document.getElementById("helpScreen").style.width = rightWidth + "px";
    document.getElementById("helpScreen").style.height = rightHeight + "px";
    document.getElementById("helpIframe").style.width = (rightWidth-40) + "px";
    document.getElementById("helpIframe").style.height = (rightHeight-40) + "px";

    canvasHS1.width=canvasWidth;
    canvasHS1.height=canvasHeight;
    canvasHS1.style.top=10;
    canvasHS1.style.left=0;
    canvasHS1t.width=canvasWidth;
    canvasHS1t.height=canvasHeight;
    canvasHS1t.style.top=10;
    canvasHS1t.style.left=0;

    canvasHS2.width=canvasWidth;
    canvasHS2.height=canvasHeight;
    canvasHS2.style.top=10;
    canvasHS2.style.left=canvasWidth;
    canvasHS2t.width=canvasWidth;
    canvasHS2t.height=canvasHeight;
    canvasHS2t.style.top=10;
    canvasHS2t.style.left=canvasWidth;

    canvasBS1.width=canvasWidth;
    canvasBS1.height=canvasHeight;
    canvasBS1.style.top=10;
    canvasBS1.style.left=2*canvasWidth+64;
    canvasBS1t.width=canvasWidth;
    canvasBS1t.height=canvasHeight;
    canvasBS1t.style.top=10;
    canvasBS1t.style.left=2*canvasWidth+64;

    canvasBS2.width=canvasWidth;
    canvasBS2.height=canvasHeight;
    canvasBS2.style.top=10;
    canvasBS2.style.left=3*canvasWidth+64;
    canvasBS2t.width=canvasWidth;
    canvasBS2t.height=canvasHeight;
    canvasBS2t.style.top=10;
    canvasBS2t.style.left=3*canvasWidth+64;

    canvasbell.width=64;
    canvasbell.height=canvasHeight;
    canvasbell.style.top=10;
    canvasbell.style.left=2*canvasWidth;

    canvasAT.width=rightWidth * 2;
    canvasAT.height=rightHeight-canvasHeight;
    canvasAT.style.top=canvasHeight+5;
    canvasAT.style.left=0;
    canvasATt.width=rightWidth * 2;
    canvasATt.height=rightHeight-canvasHeight;
    canvasATt.style.top=canvasHeight+5;
    canvasATt.style.left=0;
//    document.getElementById('ATdiv').style.top = canvasHeight + "px";
//    document.getElementById('ATdiv').style.left = 10 + "px";
//    document.getElementById('ATdiv').style.width = rightWidth + "px";
//    document.getElementById('ATdiv').style.height = (rightHeight-canvasHeight-80) + "px";

/*    document.getElementById("canvasAT").style.top = canvasHeight + "px";
    document.getElementById("canvasAT").style.left = 10 + "px";
    document.getElementById("canvasAT").style.width = (rightWidth) + "px";
    document.getElementById("canvasAT").style.height = rightHeight-canvasHeight + "px";

    document.getElementById("canvasATt").style.top = canvasHeight + "px";
    document.getElementById("canvasATt").style.left = 10 + "px";
    document.getElementById("canvasATt").style.width = (rightWidth) + "px";
    document.getElementById("canvasATt").style.height = (rightHeight-canvasHeight) + "px";
*/


//    canvasATBS.width=canvasWidth*2;
//    canvasATBS.height=0.3*canvasHeight;
//    canvasATBS.style.top=canvasHeight;
//    canvasATBS.style.left=2*canvasWidth+64;
//    canvasATBSt.width=canvasWidth*2;
//    canvasATBSt.height=0.3*canvasHeight;
//    canvasATBSt.style.top=canvasHeight;
//    canvasATBSt.style.left=2*canvasWidth+64;

//    ctxAT.fillStyle='rgba(0,0,255,0.1)';
//    ctxAT.fillRect(0,0,canvasAT.width,canvasAT.height);
//    ctxATt.fillStyle='rgba(0,128,255,0.1)';
//    ctxATt.fillRect(0,0,(canvasATt.width/2),canvasATt.height);



//    ctxATBS.fillStyle='rgba(0,255,255,0.1)';
//    ctxATBS.fillRect(0,0,canvasATBS.width,canvasATBS.height);

    radius = canvasHS1.height * 0.8;
    ctxHS1.translate(0.6*canvasHS1.height, 0.9*canvasHS1.height);
    ctxHS2.translate(0.9*canvasHS2.height, 0.9*canvasHS2.height);
    ctxBS1.translate(0.6*canvasBS1.height, 0.9*canvasBS1.height);
    ctxBS2.translate(0.9*canvasBS2.height, 0.9*canvasBS2.height);

    ctxHS1t.translate(0.6*canvasHS1t.height, 0.9*canvasHS1t.height);
    ctxHS2t.translate(0.9*canvasHS2t.height, 0.9*canvasHS2t.height);
    ctxBS1t.translate(0.6*canvasBS1t.height, 0.9*canvasBS1t.height);
    ctxBS2t.translate(0.9*canvasBS2t.height, 0.9*canvasBS2t.height);

    drawFrame(ctxHS1);
    drawFrame(ctxHS2);
    drawFrame(ctxBS1);
    drawFrame(ctxBS2);
    clearAT();
/*
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    ctx.restore();

    ctxHS1t.fillStyle='rgba(0,0,255,0.1)';
    ctxHS1t.fillRect(0,0,canvasHS1t.width,canvasHS1t.height);
    ctxHS2t.fillStyle='rgba(0,255,255,0.1)';
    ctxHS2t.fillRect(0,0,canvasHS2t.width,canvasHS2t.height);
    ctxBS1t.fillStyle='rgba(0,255,0,0.1)';
    ctxBS1t.fillRect(0,0,canvasBS1t.width,canvasBS1t.height);
    ctxBS2t.fillStyle='rgba(255,0,255,0.1)';
    ctxBS2t.fillRect(0,0,canvasBS2t.width,canvasBS2t.height);
*/
}

function drawFrame(ctx){
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

    ctx.font = "12px sans serif";
    ctx.fillStyle = "white";
    if (ctx === ctxHS1){
        ctx.textAlign = "center";
        ctx.fillText("Handstroke Pull", ctx.canvas.width*0.4, 10);
    } else if (ctx === ctxHS2) {
        ctx.textAlign = "center";
        ctx.fillText("Handstroke Check", ctx.canvas.width*0.6, 10);
    } else if (ctx === ctxBS1) {
        ctx.textAlign = "center";
        ctx.fillText("Backstroke Pull", ctx.canvas.width*0.4, 10);
    } else if (ctx === ctxBS2) {
        ctx.textAlign = "center";
        ctx.fillText("Backstroke Check", ctx.canvas.width*0.6, 10);
    }

    ctx.restore();
    ctx.beginPath();
    ctx.arc(0, 0, 0.05*radius, 0, 2*Math.PI);
    ctx.fillStyle = "white";
    ctx.fill();

    if (ctx === ctxHS1){
        starta=0;
        enda=240*Math.PI/180;
        hand=true;
    } else if (ctx === ctxHS2) {
        starta= 180*Math.PI/180;
        enda = 300*Math.PI/180;
        hand=false;
    } else if (ctx === ctxBS1) {
        starta=0;
        enda=240*Math.PI/180;
        hand= true;
    } else if (ctx === ctxBS2) {
        starta= 180*Math.PI/180;
        enda = 300*Math.PI/180;
        hand=false;
    }
    ctx.beginPath();
    ctx.arc(0, 0, 0.1*radius, starta, enda, hand);
    ctx.strokeStyle = "white";
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(0, 0, radius, starta, enda, hand);
    ctx.strokeStyle = "white";
    ctx.stroke();
}

function textBell(text){
    ctxCB.clearRect(0, 0, ctxCB.canvas.width, ctxCB.canvas.height);
    ctxCB.font="bold 16px verdana, sans-serif";
    ctxCB.textAlign="center";
    ctxCB.fillStyle = "white";
    ctxCB.fillText(text,32,32);
}

function clearBell(){
//    ctxCB.clearRect(0, 0, ctxCB.canvas.width, ctxCB.canvas.height);
    ctxCB.clearRect(0, 0, ctxCB.canvas.width, 64);

}

function clearAT(){
    ctxAT.clearRect(0, 0, ctxAT.canvas.width, ctxAT.canvas.height);
    ctxATt.clearRect(0, 0, ctxATt.canvas.width, ctxATt.canvas.height);
    ctxATt.fillStyle="rgba(204,204,230,0.1)";
    ctxATt.fillRect(0,0,canvasATt.width,canvasATt.height);
    currentATmargin=0;
    document.getElementById("canvasAT").style.marginLeft = (currentATmargin * -1) + "px";
    ctxATt.beginPath();
    ctxATt.rect(1,1,(ctxATt.canvas.width/2)-1, ctxATt.canvas.height-ATbottomMargin);
    var halfHeight = (ctxATt.canvas.height-ATbottomMargin)/2;
    ctxATt.moveTo(0,halfHeight);
    ctxATt.lineTo(ctxATt.canvas.width,halfHeight);
    ctxATt.lineWidth=1;
    ctxATt.strokeStyle="rgba(240,240,240,0.6)";
    ctxATt.stroke();
}

function drawBell(angle) {
    var DEG2RAD = Math.PI/180;
    var bx = 32 + (Math.sin((angle - 80) * DEG2RAD) * 10);
    var by = 32 + (Math.cos((angle - 80) * DEG2RAD) * 10);
    var cx = 32 + (Math.sin((angle -  5) * DEG2RAD) * 15);
    var cy = 32 + (Math.cos((angle -  5) * DEG2RAD) * 15);
    var dx = 32 + (Math.sin((angle - 40) * DEG2RAD) * 30);
    var dy = 32 + (Math.cos((angle - 40) * DEG2RAD) * 30);
    var ex = 32 + (Math.sin((angle + 80) * DEG2RAD) * 10);
    var ey = 32 + (Math.cos((angle + 80) * DEG2RAD) * 10);
    var fx = 32 + (Math.sin((angle +  5) * DEG2RAD) * 15);
    var fy = 32 + (Math.cos((angle +  5) * DEG2RAD) * 15);
    var gx = 32 + (Math.sin((angle + 40) * DEG2RAD) * 30);
    var gy = 32 + (Math.cos((angle + 40) * DEG2RAD) * 30);
    ctxCB.clearRect(0, 0, ctxCB.canvas.width, 64);
    ctxCB.beginPath();
    ctxCB.moveTo(32, 32);
    ctxCB.bezierCurveTo(bx, by, cx, cy, dx, dy);
    ctxCB.moveTo(32, 32);
    ctxCB.bezierCurveTo(ex, ey, fx, fy, gx, gy);
    ctxCB.lineWidth = 4;
//    ctxCB.strokeStyle = "#303030";
    ctxCB.strokeStyle = "white";
    ctxCB.stroke();
}

function drawAccel(ctx, pos1, pos2, length, clearonly) {

    var scaleSize = 120/(ROIL-ROIU); // this is how much the position needs to be scaled
    var starta, enda, hand
    if (pos1 <= 180) pos1=-30+((pos1-ROIU)*scaleSize);
    if (pos2 <= 180) pos2=-30+((pos2-ROIU)*scaleSize);
    if (pos1 > 180) pos1=270+((pos1-(360-ROIL))*scaleSize);
    if (pos2 > 180) pos2=270+((pos2-(360-ROIL))*scaleSize);

    if (ctx === ctxHS1){ // handstroke down
        pos1=(pos1+180)*Math.PI/180;
        pos2=(pos2+180)*Math.PI/180;
        starta=0;
        enda=240*Math.PI/180;
        hand=true;

    } else if (ctx === ctxHS2) { // handstroke up
        pos1=(pos1-180)*Math.PI/180;
        pos2=(pos2-180)*Math.PI/180;
        starta= 180*Math.PI/180;
        enda = 300*Math.PI/180;
        hand=false;

    } else if (ctx === ctxBS1) { // backstroke down
        pos1=(180-pos1)*Math.PI/180;
        pos2=(180-pos2)*Math.PI/180;
        starta=0;
        enda=240*Math.PI/180;
        hand= true;

    } else if (ctx === ctxBS2) { // backstroke up
        pos1=(180-pos1)*Math.PI/180;
        pos2=(180-pos2)*Math.PI/180;
        starta= 180*Math.PI/180;
        enda = 300*Math.PI/180;
        hand=false;
    }
//clear section
    ctx.beginPath();
    ctx.moveTo(0,0);
    ctx.rotate(pos1);
    ctx.moveTo(0, 0.15*radius);
    ctx.lineTo(0, 0.92*radius);
    ctx.rotate(pos2-pos1+0.03);//0.03 is 3 degrees over for clear
    ctx.arc(0,0,0.92*radius,(0.5*Math.PI)-(pos2-pos1),(0.5*Math.PI),false);
//    ctx.lineTo(0, length);
    ctx.lineTo(0, 0.15*radius);
    ctx.rotate(-pos2-0.03);
    ctx.fillStyle= BGCOLOUR;
    ctx.fill();

    if (clearonly == false){
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.rotate(pos1);
        ctx.moveTo(0, 0.15*radius);
        ctx.lineTo(0, (0.15*radius)+length);
        ctx.rotate(pos2-pos1);
        ctx.arc(0,0,(0.15*radius)+length,(0.5*Math.PI)-(pos2-pos1),(0.5*Math.PI),false);
    //    ctx.lineTo(0, length);
        ctx.lineTo(0, 0.15*radius);
        ctx.rotate(-pos2);
        ctx.fillStyle= "white";
        ctx.fill();
    }
    ctx.beginPath(); // redraw centre axles
    ctx.arc(0, 0, 0.05*radius, 0, 2*Math.PI);
    ctx.fillStyle = "white";
    ctx.fill();
    ctx.beginPath();
    ctx.arc(0, 0, 0.1*radius, starta, enda, hand);
    ctx.strokeStyle = "white";
    ctx.stroke();
}

function drawAccelT(ctx, pos1, pos2, length) {
    var scaleSize = 120/(ROIL-ROIU); // this is how much the position needs to be scaled
    if (pos1 <= 180) pos1=-30+((pos1-ROIU)*scaleSize);
    if (pos2 <= 180) pos2=-30+((pos2-ROIU)*scaleSize);
    if (pos1 > 180) pos1=270+((pos1-(360-ROIL))*scaleSize);
    if (pos2 > 180) pos2=270+((pos2-(360-ROIL))*scaleSize);

    if (ctx === ctxHS1t){ // handstroke down
        pos1=(pos1+180)*Math.PI/180;
        pos2=(pos2+180)*Math.PI/180;

    } else if (ctx === ctxHS2t) { // handstroke up
        pos1=(pos1-180)*Math.PI/180;
        pos2=(pos2-180)*Math.PI/180;

    } else if (ctx === ctxBS1t) { // backstroke down
        pos1=(180-pos1)*Math.PI/180;
        pos2=(180-pos2)*Math.PI/180;

    } else if (ctx === ctxBS2t) { // backstroke up
        pos1=(180-pos1)*Math.PI/180;
        pos2=(180-pos2)*Math.PI/180;
    }
    ctx.beginPath();
    ctx.moveTo(0,0);
    ctx.rotate(pos1);
    ctx.moveTo(0, 0.15*radius);
    ctx.lineTo(0, (0.15*radius)+length);
    ctx.rotate(pos2-pos1);
    ctx.arc(0,0,(0.15*radius)+length,(0.5*Math.PI)-(pos2-pos1),(0.5*Math.PI),false);
//    ctx.lineTo(0, length);
    ctx.lineTo(0, 0.15*radius);
    ctx.rotate(-pos2);
    ctx.fillStyle="rgba(240,240,0,0.6)";
    ctx.fill();

}

function drawTDCs(){

    drawTarget();
    if ((currentStatus & GRIDLINESDISPLAYED) == 0) return;
    var stepSize=null;
    var currentStep=0;
    drawTDC(ctxHS1t,0,"TDC");
    drawTDC(ctxHS2t,0,"TDC");
    drawTDC(ctxBS1t,0,"TDC");
    drawTDC(ctxBS2t,0,"TDC");
    if (ROIL-ROIU >=60){
        stepSize=20;
        currentStep=20;
    } else {
        stepSize=10;
        currentStep=10;
    }
    while (currentStep <= ROIL){
        drawTDC(ctxHS1t,currentStep,currentStep.toString());
        drawTDC(ctxHS2t,currentStep,currentStep.toString());
        drawTDC(ctxBS1t,currentStep,currentStep.toString());
        drawTDC(ctxBS2t,currentStep,currentStep.toString());
        currentStep+=stepSize;
    }
}

function drawTarget(){
    var scaleSize = 120/(ROIL-ROIU);

    if (targetAngleHand !== null) {
        var angle = targetAngleHand;
        var zero = -30+((angle-ROIU)*scaleSize);
        zero = (zero+180)*Math.PI/180;
        ctxHS1t.beginPath();
        ctxHS1t.moveTo(0,0);
        ctxHS1t.rotate(zero);
        ctxHS1t.moveTo(0, 0.15*radius);
        ctxHS1t.lineTo(0, 0.95*radius);
        ctxHS1t.lineWidth=4;
        ctxHS1t.strokeStyle="rgba(0,240,0,0.9)";
        ctxHS1t.rotate(-zero);
        ctxHS1t.stroke();
        ctxHS1t.lineWidth=1;

        zero = -30+((angle-ROIU)*scaleSize);
        zero = (180-zero)*Math.PI/180;
        ctxBS2t.beginPath();
        ctxBS2t.moveTo(0,0);
        ctxBS2t.rotate(zero);
        ctxBS2t.moveTo(0, 0.15*radius);
        ctxBS2t.lineTo(0, 0.95*radius);
        ctxBS2t.lineWidth=4;
        ctxBS2t.strokeStyle="rgba(0,240,0,0.9)";
        ctxBS2t.rotate(-zero);
        ctxBS2t.stroke();
        ctxBS2t.lineWidth=1;

    }
    if (targetAngleBack !== null) {

        angle = targetAngleBack;
        zero = 270+(((360-angle)-(360-ROIL))*scaleSize);
        zero = (zero-180)*Math.PI/180;
        ctxHS2t.beginPath();
        ctxHS2t.moveTo(0,0);
        ctxHS2t.rotate(zero);
        ctxHS2t.moveTo(0, 0.15*radius);
        ctxHS2t.lineTo(0, 0.95*radius);
        ctxHS2t.lineWidth=4;
        ctxHS2t.strokeStyle="rgba(0,240,0,0.9)";
        ctxHS2t.rotate(-zero);
        ctxHS2t.stroke();
        ctxHS2t.lineWidth=1;

        angle = targetAngleBack;
        zero = 270+(((360-angle)-(360-ROIL))*scaleSize);
        zero = (180-zero)*Math.PI/180;
        ctxBS1t.beginPath();
        ctxBS1t.moveTo(0,0);
        ctxBS1t.rotate(zero);
        ctxBS1t.moveTo(0, 0.15*radius);
        ctxBS1t.lineTo(0, 0.95*radius);
        ctxBS1t.lineWidth=4;
        ctxBS1t.strokeStyle="rgba(0,240,0,0.9)";
        ctxBS1t.rotate(-zero);
        ctxBS1t.stroke();
        ctxBS1t.lineWidth=1;

    }
}


function drawTDC(ctx,angle,text){
    var scaleSize = 120/(ROIL-ROIU); // this is how much the position needs to be scaled
    var zero = 0;
    var rot=0;
    var alt=false;

    if (ctx === ctxHS1t){ // handstroke down
        zero = -30+((angle-ROIU)*scaleSize);
        zero = (zero+180)*Math.PI/180;
        rot = 3.1415/2;

    } else if (ctx === ctxHS2t) { // handstroke up
        zero = 270+(((360-angle)-(360-ROIL))*scaleSize);
        zero = (zero-180)*Math.PI/180;
        rot = -3.1415/2;
        alt = true;

    } else if (ctx === ctxBS1t) { // backstroke down
        zero = 270+(((360-angle)-(360-ROIL))*scaleSize);
        zero = (180-zero)*Math.PI/180;
        rot = 3.1415/2;

    } else if (ctx === ctxBS2t) { // backstroke up
        zero = -30+((angle-ROIU)*scaleSize);
        zero = (180-zero)*Math.PI/180;
        rot = -3.1415/2;
        alt=true;
    }
    ctx.beginPath();
    ctx.moveTo(0,0);
    ctx.rotate(zero);
    ctx.moveTo(0, 0.15*radius);
    ctx.lineTo(0, 0.95*radius);
    ctx.strokeStyle="rgb(255,100,100)";
    ctx.rotate(-zero);
    ctx.stroke();

    ctx.font = "12px sans serif";
    ctx.fillStyle = "rgb(255,100,100)";
    ctx.rotate(zero+rot);
    if (alt == false) {
        ctx.textAlign = "end";
        ctx.fillText(text, 0.95*radius, -3);
    } else {
        ctx.textAlign = "start";
        ctx.fillText(text, -0.95*radius, -3);
    }
    ctx.rotate(-zero-rot);
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

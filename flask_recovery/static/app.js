const socket = io();

// 선택 상태
let eggSelected = false;
let greenSelected = false;

// ⭐ 현재 남은 시간을 저장할 전역 변수 (360초 = 6분)
let timerInterval = null;
let currentSec = 360; 

// 버튼 토글
function toggleEgg() {
    eggSelected = !eggSelected;
    document.getElementById("btn_egg").classList.toggle("selected", eggSelected);
}

function toggleGreen() {
    greenSelected = !greenSelected;
    document.getElementById("btn_green").classList.toggle("selected", greenSelected);
}

// STOP (일시 정지)
function pressStop() {
    socket.emit("stop_signal", true);
    console.log("🛑 STOP pressed");
    
    pauseTimer(); 
    document.getElementById("progress_text").innerHTML = "📡 상태 : 일시 정지됨";
}

// 🔄 RECOVERY
function pressRecovery() {
    socket.emit("recovery_signal", true);
    console.log("🔄 RECOVERY pressed - System Reset");
    
    resetSystem();
    document.getElementById("progress_text").innerHTML = "📡 상태 (0) : 초기 대기 중";
}

// START (새로운 주문 시작)
function pressStart() {
    let mode = 0;
    if (eggSelected && greenSelected) mode = 3;
    else if (eggSelected) mode = 1;
    else if (greenSelected) mode = 2;

    socket.emit("mode_select", {mode: mode});
    socket.emit("start_signal", true);

    startNewTimer(); // 타이머를 6분부터 새로 시작

    // 선택 초기화
    eggSelected = false;
    greenSelected = false;
    document.getElementById("btn_egg").classList.remove("selected");
    document.getElementById("btn_green").classList.remove("selected");
}


/* ---------------- 타이머 ---------------- */
// 1. 타이머를 6분(360초)부터 새로 시작하는 함수
function startNewTimer() {
    currentSec = 360; // 시간을 6분으로 리셋
    resumeTimer();
}

// 2. 현재 시간(currentSec)부터 타이머를 작동시키는 핵심 함수
function resumeTimer() {
    clearInterval(timerInterval);

    timerInterval = setInterval(() => {
        const m = String(Math.floor(currentSec / 60)).padStart(2, '0');
        const s = String(currentSec % 60).padStart(2, '0');

        document.getElementById("timer").innerText = `${m}:${s}`;

        if (currentSec <= 0) {
            clearInterval(timerInterval);
        }
        currentSec--; 
    }, 1000);
}

// 3. 타이머 작동만 중지하는 함수 (pause)
function pauseTimer() {
    clearInterval(timerInterval);
}

// 4. 시스템 완전 초기화 함수 (Recovery 버튼 전용)
function resetSystem() {
    clearInterval(timerInterval);
    currentSec = 360;
    document.getElementById("timer").innerText = "06:00";
}


/* ----------- ROS 진행 상태 ----------- */
socket.on("progress_update", (data) => {
    let msg = "";
    switch (data.state) {
        case 1: msg = "냄비 놓는 중.."; break;
        case 2: msg = "물 따르는 중.."; break;
        case 3: msg = "면 넣는 중.."; break;
        case 4: msg = "소스 넣는 중.."; break;
        default: msg = "알 수 없음";
    }

    document.getElementById("progress_text").innerHTML =
        `📡 상태 (${data.state}) : ${msg}`;
});

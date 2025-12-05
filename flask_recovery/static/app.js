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
    document.getElementById("progress_text").innerHTML = "정지 중";
}

// 🔄 RECOVERY (완전 초기화 기능으로 변경) ⭐ 수정됨 ⭐
function pressRecovery() {
    socket.emit("recovery_signal", true);
    console.log("🔄 RECOVERY pressed - System Reset");
    
    resetSystem();
    document.getElementById("progress_text").innerHTML = "리커버리 중";

    // ⭐ 여기서만 토핑 선택 초기화 ⭐
    eggSelected = false;
    greenSelected = false;

    document.getElementById("btn_egg").classList.remove("selected");
    document.getElementById("btn_green").classList.remove("selected");
}

// START (새로운 주문 시작)
function pressStart() {
    let mode = 0;
    if (eggSelected && greenSelected) mode = 3;
    else if (eggSelected) mode = 1;
    else if (greenSelected) mode = 2;

    socket.emit("mode_select", {mode: mode});
    socket.emit("start_signal", true);

    // ⭐ 모드별 타이머 실행
    if (mode === 0) startNewTimer0();
    if (mode === 1) startNewTimer1();
    if (mode === 2) startNewTimer2();
    if (mode === 3) startNewTimer3();

    // ❌ 삭제해라 — START 눌러도 선택 유지해야 하니까
    // eggSelected = false;
    // greenSelected = false;
    // document.getElementById("btn_egg").classList.remove("selected");
    // document.getElementById("btn_green").classList.remove("selected");
}


/* ---------------- 타이머 ---------------- */


function startNewTimer0() {
    currentSec = 260; // 시간을 6분으로 리셋
    resumeTimer();
}
function startNewTimer1() {
    currentSec = 295; // 시간을 6분으로 리셋
    resumeTimer();
}
function startNewTimer2() {
    currentSec = 295; // 시간을 6분으로 리셋
    resumeTimer();
}
function startNewTimer3() {
    currentSec = 330; // 시간을 6분으로 리셋
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
            // ⭐ NEW: 서버에 end_signal 전송 ⭐
            socket.emit("end_signal", true);
            console.log("🍜 end_signal emitted to server!");

            return;
        }

        currentSec--;
    }, 1000);
}

// 3. 타이머 작동만 중지하는 함수 (pause)
function pauseTimer() {
    clearInterval(timerInterval); // 타이머 작동만 중지하고, currentSec 값은 유지
}

// 🛑 4. 시스템 완전 초기화 함수 (Recovery 버튼 전용) ⭐ 새로 추가 ⭐
function resetSystem() {
    clearInterval(timerInterval); // 작동 중인 타이머 정지
    currentSec = 360; // 남은 시간 변수를 6분으로 초기화
    document.getElementById("timer").innerText = "06:00"; // 화면 표시 초기화
}


/* ----------- ROS 진행 상태 ----------- */
socket.on("progress_update", (data) => {
    let msg = "";
    switch (data.state) {
        case 0: msg = "대기중"; break;    
        case 1: msg = "냄비 놓는 중.."; break;
        case 2: msg = "물 따르는 중.."; break;
        case 3: msg = "면 넣는 중.."; break;
        case 4: msg = "스프/고명 넣는 중.."; break;
        case 5: msg = "라면 끓이는 중.."; break;
        case 6: msg = "물 끓이는 중.."; break;

        /* 🛑⭐ 재료 소진 경고 로직 ⭐🛑 */
        case 7:
            document.getElementById("progress_text").style.color = "red";
            document.getElementById("progress_text").innerHTML =
                "⚠ 재료가 소진되었습니다!";
            
            pauseTimer();
            alert("⚠ 재료가 소진되었습니다! \n재료를 채워주세요.");

            return; 
        case 8: msg = "라면이 완성되었습니다! 🍜✨"; break;
    }

    // 정상 동작일 때 기본 메시지 출력
    const progress = document.getElementById("progress_text");
    progress.style.color = "#000";  
    progress.innerHTML = `${msg}`;
});


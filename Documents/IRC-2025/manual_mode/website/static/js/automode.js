// Wait for DOM to be fully loaded
document.addEventListener("DOMContentLoaded", () => {
  let timer;
  let seconds = 0;
  let isRunning = false;

  // Get DOM elements
  const timerDisplay = document.getElementById("timer");
  const startButton = document.getElementById("start");
  const stopButton = document.getElementById("stop");
  const restartButton = document.getElementById("restart");

  // Format time as HH:MM:SS
  function formatTime(sec) {
    const hrs = Math.floor(sec / 3600);
    const mins = Math.floor((sec % 3600) / 60);
    const secs = sec % 60;
    return `${hrs.toString().padStart(2, "0")}:${mins
      .toString()
      .padStart(2, "0")}:${secs.toString().padStart(2, "0")}`;
  }

  // Update timer display
  function updateDisplay() {
    timerDisplay.textContent = formatTime(seconds);
  }

  // Start timer
  function startTimer() {
    if (!isRunning) {
      timer = setInterval(() => {
        seconds++;
        updateDisplay();
      }, 1000);
      isRunning = true;
      startButton.disabled = true;
      stopButton.disabled = false;
      restartButton.disabled = false;
    }
  }

  // Stop timer
  function stopTimer() {
    clearInterval(timer);
    isRunning = false;
    startButton.disabled = false;
    stopButton.disabled = true;
  }

  // Restart timer
  function restartTimer() {
    clearInterval(timer);
    seconds = 0;
    isRunning = false;
    updateDisplay();
    startButton.disabled = false;
    stopButton.disabled = true;
    restartButton.disabled = true;
  }

  // Event listeners
  startButton.addEventListener("click", startTimer);
  stopButton.addEventListener("click", stopTimer);
  restartButton.addEventListener("click", restartTimer);

  // Initialize display
  updateDisplay();
});

// Wait for the DOM to be fully loaded
document.addEventListener("DOMContentLoaded", function () {
  // Get the main camera image
  const mainCamera = document.querySelector(".camera-main");

  // Get all the secondary camera images
  const secondaryCameras = document.querySelectorAll(".b2 img");

  // Add click event listeners to all secondary cameras
  secondaryCameras.forEach((camera) => {
    camera.addEventListener("click", function () {
      // Store the main camera's current source and alt
      const mainSrc = mainCamera.src;
      const mainAlt = mainCamera.alt;

      // Update main camera with clicked image's source and alt
      mainCamera.src = this.src;
      mainCamera.alt = this.alt;

      // Update clicked image with previous main camera's source and alt
      this.src = mainSrc;
      this.alt = mainAlt;
    });
  });
});

const rover = document.getElementById("rover");
const compassValueEl = document.getElementById("compass-value");

function updateCompass() {
  // Generate a random compass value (0° to 360°)
  const compassValue = Math.floor(Math.random() * 360);
  compassValueEl.textContent = `${compassValue}°`;

  // Calculate the position of the rover based on the compass value
  const angleInRadians = (compassValue * Math.PI) / 180;
  const radius = 120; // Radius of the circle in pixels
  const x = Math.cos(angleInRadians) * radius;
  const y = Math.sin(angleInRadians) * radius;

  // Update rover's position
  rover.style.left = `calc(50% + ${x}px)`;
  rover.style.top = `calc(50% - ${y}px)`;
}

// Update compass value and rover position every second
setInterval(updateCompass, 1000);

// Function to update the arrow count
function updateArrowCount(count) {
  document.getElementById("arrowCount").textContent = count;
}

// Function to update the last detection details
function updateLastDetection(data) {
  document.getElementById("direction").textContent = data.direction;
  document.getElementById("gpsCoordinates").textContent = data.gpsCoordinates;
  document.getElementById("cardinalLocation").textContent =
    data.cardinalLocation;
}

/* Project Name: Lab 4
 * Author: Oluwagbemiga Oloyede, Kashish Garg, Dhyey Shah
 * License: You may use, distribute and modify this code under the terms of the GNU GPLv3.0 license
 * Name: 4.2
 */

const char htmlPage[] PROGMEM = R"===(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot Car Controller</title>
  <style>
    body {
      display: flex;
      flex-direction: column;
      align-items: center;
      font-family: Arial, sans-serif;
      background-color: #f2f2f2;
      margin: 0;
      padding: 20px;
    }

    .interface {
      display: flex;
      flex-direction: row;
      padding: 20;
      justify-content: space-between;
      /* transform: rotate(-270deg); */
    }

    h1 {
      color: #333;
    }

    .controls {
      display: grid;
      grid-template-areas:
        ". up ."
        "left stop right"
        ". down .";
      gap: 20px;
      /* justify-content: center;
      /* transform: rotate(-90deg); */
    }

    .controls button {
      width: 80px;
      height: 80px;
      font-size: 18px;
      font-weight: bold;
      cursor: pointer;
      border: none;
      border-radius: 10px;
      color: #fff;
      transition: 0.3s;
    }

    .controls-div {
      /* align-items: center; */
      display: flex;
      padding: 20px;
      align-items: center;
      justify-content: center;
      margin-right: 120px;

    }

    .up {
      grid-area: up;
      background-color: #4CAF50;
    }

    .down {
      grid-area: down;
      background-color: #e74c3c;
    }

    .left {
      grid-area: left;
      background-color: #3498db;
    }

    .right {
      grid-area: right;
      background-color: #f39c12;
    }

    .stop {
      grid-area: stop;
      background-color: #555;
    }

    .speed-control {
      margin-top: 20px;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      /* transform: rotate(-180deg); */
    }

    .speed-control label {
      margin-bottom: 10px;
      font-size: 18px;
      color: #333;
    }

    .speed-control input[type="range"] {
      width: 300px;
    }

    @media (max-width: 600px) {
      .speed-control {
        width: 90%;
        /* adjust slider width to fit smaller screens */
      }

      body {
        padding: 20px;
      }

      .interface {
        display: flex;
        flex-direction: row;
        padding: 20;
        justify-content: space-between;
        transform: rotate(-270deg);
        /* padding-top: 60vh; */
        padding-left: 50vh;
      }

      .controls button {
        width: 65px;
        height: 65px;
        font-size: 18px;
        font-weight: bold;
        cursor: pointer;
        border: none;
        border-radius: 10px;
        color: #fff;
        transition: 0.3s;
      }

      .controls-div {
        /* align-items: center; */
        display: flex;
        padding: 10px;
        align-items: center;
        justify-content: center;
        margin-right: 20px;

      }

      #header {
        display: none;
      }
    }

    /* @media (max-height: 376) {
            .speed-control input[type="range"] {
                width: 10px;
                height: 10px;
            }
            #header {
                display: none;
            }
            
        } */
  </style>
</head>

<body>
  <div id="header">
    <h1 id="header-elem">Robot Car Controller</h1>
  </div>
  <div class="interface">
    <!-- control -->
    <div class="controls-div">
      <div class="controls">
        <button class="up" onclick="sendCommand('1')">Up</button>
        <button class="left" onclick="sendCommand('2')">Left</button>
        <button class="stop" onclick="sendCommand('0')">Stop</button>
        <button class="right" onclick="sendCommand('3')">Right</button>
        <button class="down" onclick="sendCommand('-1')">Down</button>
      </div>
    </div>
    <!-- Speed control -->
    <div class="speed-control">
      <label for="speed">Speed Control</label>
      <input type="range" id="speed" min="0" max="100" value="50" oninput="updateSpeed(this.value)">
      <span id="speed-display">50</span>%
    </div>
  </div>



  <script>
    function sendCommand(command) {
      console.log(`sending command: ${command}`);
      var xhttp = new XMLHttpRequest();
      var str = "setDirection?dir="
      var res = str.concat(command);
      xhttp.open("GET", res, true);
      xhttp.send();
    }

    function updateSpeed(speed) {
      var xhttp = new XMLHttpRequest();
      var spd = speed;
      // var str = "speed?val=";
      var str = "setSpeed?value="
      var res = str.concat(speed);
      xhttp.open("GET", res, true);
      xhttp.send();
      document.getElementById('speed-display').textContent = speed;
      console.log(`setting speed to: ${speed}`);
    }
  </script>
</body>
</html>
)===";

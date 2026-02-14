const container = document.getElementById("gyro3d");

// Scène
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
var cube_color = 0x00ff99

// Caméra
const camera = new THREE.PerspectiveCamera(
    60,
    container.clientWidth / container.clientHeight,
    0.1,
    100
);
camera.position.set(2, 2, 2);
camera.lookAt(0, 0, 0);

// Rendu
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(container.clientWidth, container.clientHeight);
container.appendChild(renderer.domElement);

// Repère XYZ
const axes = new THREE.AxesHelper(1.5);
scene.add(axes);

// Lumière
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(3, 3, 3);
scene.add(light);

// Objet (cube = robot)
const geometry = new THREE.BoxGeometry(1, 0.2, 0.5);
const material = new THREE.MeshStandardMaterial({ color: cube_color });
const robot = new THREE.Mesh(geometry, material);
scene.add(robot);

const arrowLength = 0.2;
const arrowColor = 0xff0000; // rouge
const arrowDir = new THREE.Vector3(1, 0, 0); // pointe vers +X
arrowDir.normalize();

// Positionner la flèche à l'avant du cube
const arrowOrigin = new THREE.Vector3(0.5, 0, 0); // moitié de la longueur du cube en X
const arrowHelper = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLength, arrowColor);

robot.add(arrowHelper);

// Rendu continu
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}
animate();

var battery = 0
// Mise à jour de la page et orientation du cube
async function updateData() {
    try {
        const response = await fetch("/data");
        const data = await response.json();

        // Mise à jour des spans
        document.getElementById("battery").textContent = data.battery;
        document.getElementById("gyro_x").textContent = data.gyro_x;
        document.getElementById("gyro_y").textContent = data.gyro_y;
        document.getElementById("gyro_z").textContent = data.gyro_z;
        document.getElementById("accel_x").textContent = data.accel_x;
        document.getElementById("accel_y").textContent = data.accel_y;
        document.getElementById("accel_z").textContent = data.accel_z;
        document.getElementById("temp").textContent = data.temp;
        document.getElementById("battery_percent").textContent = data.battery_percent;
        document.getElementById("vitesse").textContent = data.vitesse;
        battery = data.battery_percent
        document.getElementById("sensibilite").textContent = data.sensibilite
        document.getElementById("distance_front").textContent = data.distance_front;

        action_active = document.getElementById("action_active");
        if (data.active) {
            action_active.textContent = "Oui";
            action_active.style.color = "green";
        } else {
            action_active.textContent = "Non";
            action_active.style.color = "red";
        };

        // Orientation du cube
        const gx = parseFloat(data.gyro_x) * Math.PI / 180;
        const gy = parseFloat(data.gyro_y) * Math.PI / 180;
        const gz = parseFloat(data.gyro_z) * Math.PI / 180;

        robot.rotation.set(gx, 0, gy);

    } catch (e) {
        console.warn("Données indisponibles");
    }
}

setInterval(updateData, 50); // 20 Hz
updateData();

document.getElementById("start").addEventListener("click", () => {
    fetch("/start", { method: "POST" })
        .then(response => response.json())
        .then(data => console.log(data));
});

document.getElementById("stop").addEventListener("click", () => {
    fetch("/stop", { method: "POST" })
        .then(response => response.json())
        .then(data => console.log(data));
});



document.getElementById("vitesse_input").addEventListener("change", (event) => {
    const vitesse = parseInt(event.target.value);
    fetch(`/set_vitesse?vitesse=${vitesse}`, { method: "POST" })
        .then(response => response.json())
        .then(data => console.log(data));
});

document.getElementById("sensibilite_input").addEventListener("change", (event) => {
    const sensitivity = parseInt(event.target.value);
    fetch(`/set_sensitivity?sensitivity=${sensitivity}`, { method: "POST" })
        .then(response => response.json())
        .then(data => console.log(data));
});

function updateBatteryBar() {
    var battery_bar = document.getElementById("battery_fill");
    battery_bar.style.width = battery+"%";



    if (battery >= 50) {
        battery_bar.style.backgroundColor="green";
    } else if (battery >= 25) {
        battery_bar.style.backgroundColor="orange";
    } else {
        battery_bar.style.backgroundColor="red";
    };


}

setInterval(updateBatteryBar, 100); // 20 Hz
updateData();

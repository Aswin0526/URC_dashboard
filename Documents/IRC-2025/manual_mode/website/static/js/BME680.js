async function fetchBME680() {
    try {
        const response = await fetch('http://192.168.1.110:5000/science-subsystem/BME680');
        const data = await response.json()
        console.log(data)
        document.getElementById("BME680-temperature").textContent = `${data.temperature}`
        document.getElementById("BME680-humidity").textContent = `${data.humidity}`
        document.getElementById("BME680-pressure").textContent = `${data.pressure}`
        document.getElementById("BME680-voc").textContent = `${data.voc}`
    } catch (error) {
        console.error("Error fetching BME680", error);
    }
}

setInterval(fetchBME680, 1000);
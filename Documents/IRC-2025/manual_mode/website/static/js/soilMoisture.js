async function fetchSoilMoisture() {
    try {
        const response = await fetch('http://192.168.1.110:5000/science-subsystem/soil-moisture');
        var data = await response.json()
        data = data.soil_moisture
        console.log(data)
        document.getElementById("soil-moisture").textContent = `${data}%`
        const newDashArray = `${data}, ${100-data}`
        console.log(newDashArray)
        document.getElementById("soil-moisture-circle").setAttribute("stroke-dasharray", newDashArray)
    } catch (error) {
        console.error("Error fetching soil moisture data", error);
    }
}

setInterval(fetchSoilMoisture, 1000);
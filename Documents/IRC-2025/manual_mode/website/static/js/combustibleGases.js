async function fetchCombustibleGases() {
    try {
        const response = await fetch('http://192.168.1.110:5000/science-subsystem/combustible-gases');
        var data = await response.json()
        data = data.combustible_gases
        console.log(data)
        document.getElementById("combustible-gases").textContent = `${data}%`
        const newDashArray = `${data}, ${100-data}`
        console.log(newDashArray)
        document.getElementById("combustible-gases-circle").setAttribute("stroke-dasharray", newDashArray)
    } catch (error) {
        console.error("Error fetching combustible gases", error);
    }
}

setInterval(fetchCombustibleGases, 1000);
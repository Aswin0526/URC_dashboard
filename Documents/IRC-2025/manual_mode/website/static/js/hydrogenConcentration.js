// async function fetchHydrogenConcentration() {
//     try {
//         const response = await fetch('http://127.0.0.1:5000/science-subsystem/hydrogen-concentration');
//         const data = await response.json()
//         console.log(data)
//         document.getElementById("hydrogen-concentration").textContent = `${data.hydrogen_concentration} ppm`
//     } catch (error) {
//         console.error("Error fetching hydrogen concentration", error);
//     }
// }

// setInterval(fetchHydrogenConcentration, 1000);


document.addEventListener('DOMContentLoaded', function () {
    // Initialize Chart.js on the hydrogen-concentration canvas
    const ctx = document.getElementById('hydrogen-concentration').getContext('2d');

    // Define initial data and config for Chart.js line chart
    const hydrogen_concentration = {
        labels: [],
        datasets: [{
            label: 'Hydrogen Conc.(ppm)',
            borderColor: '#00ff00',
            backgroundColor: 'rgba(0, 255, 0, 0.2)',
            data: []
        }]
    };

    const hydrogenConcentrationConfig = {
        type: 'line',
        data: hydrogen_concentration,
        options: {
            responsive: true,
            scales: {
                x: {
                    title: { display: true, text: 'Time (MM:SS)' }
                },
                y: {
                    title: { display: true, text: 'Hydrogen Conc.(ppm)' }
                }
            }
        }
    };

    // Create the line chart instance
    const hydrogenConcentrationChart = new Chart(ctx, hydrogenConcentrationConfig);

    // Function to fetch and update air quality data
    async function updateHydrogenConcentration() {
        try {
            // Assuming a fetch API is available to get hydrogen concentration data
            const response = await fetch('http://:192.168.1.110:5000/science-subsystem/hydrogen-concentration'); // Replace with your actual endpoint
            const data = await response.json();
            const hydrogenConcentration = data.hydrogen_concentration; // Assuming the data has a property called 'hydrogen_concentration'

            const date = new Date()
            const seconds = date.getSeconds()
            const minutes = date.getMinutes()

            hydrogen_concentration.labels.push(`${minutes}:${seconds}`);
            hydrogen_concentration.datasets[0].data.push(hydrogenConcentration);

            // Remove old data to keep chart clean
            if (hydrogen_concentration.labels.length > 9) {
                hydrogen_concentration.labels.shift();
                hydrogen_concentration.datasets[0].data.shift();
            }

            // Update the chart
            hydrogenConcentrationChart.update();
        } catch (error) {
            console.error("Error fetching hydrogen concentration :", error);
        }
    }

    // Update chart data at regular intervals
    setInterval(updateHydrogenConcentration, 1000); // Update every 1 second
});
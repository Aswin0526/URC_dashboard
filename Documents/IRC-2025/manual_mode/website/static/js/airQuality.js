document.addEventListener("DOMContentLoaded", function () {
  // Initialize Chart.js on the air-quality canvas
  const ctx = document.getElementById("air-quality").getContext("2d");

  // Define initial data and config for Chart.js line chart
  const airQualityData = {
    labels: [],
    datasets: [
      {
        label: "Air Quality (ppm)",
        borderColor: "#ff8419",
        backgroundColor: "rgba(255, 132, 25, 0.2)",
        data: [],
      },
    ],
  };

  const airQualityConfig = {
    type: "line",
    data: airQualityData,
    options: {
      responsive: true,
      scales: {
        x: {
          title: { display: true, text: "Time (MM:SS)" },
        },
        y: {
          title: { display: true, text: "Air Quality (ppm)" },
        },
      },
    },
  };

  // Create the line chart instance
  const airQualityChart = new Chart(ctx, airQualityConfig);

  // Function to fetch and update air quality data
  async function updateAirQualityData() {
    try {
      // Assuming a fetch API is available to get air quality data
      const response = await fetch("/science-subsystem/air-quality"); // Replace with your actual endpoint
      const data = await response.json();
      const airQualityValue = data.air_quality; // Assuming the data has a property called 'air_quality'

      const date = new Date();
      const seconds = date.getSeconds();
      const minutes = date.getMinutes();

      airQualityData.labels.push(`${minutes}:${seconds}`);
      airQualityData.datasets[0].data.push(airQualityValue);

      // Remove old data to keep chart clean
      if (airQualityData.labels.length > 9) {
        airQualityData.labels.shift();
        airQualityData.datasets[0].data.shift();
      }

      // Update the chart
      airQualityChart.update();
    } catch (error) {
      console.error("Error fetching air quality data:", error);
    }
  }

  // Update chart data at regular intervals
  setInterval(updateAirQualityData, 1000); // Update every 1 second
});

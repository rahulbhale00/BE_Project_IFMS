"use client";

import { useEffect, useState } from "react";
import { Line } from "react-chartjs-2";
import {
  Chart as ChartJS,
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import annotationPlugin from "chartjs-plugin-annotation";
import ChartDataLabels from "chartjs-plugin-datalabels";

// Register Chart.js components
ChartJS.register(
  LineElement,
  CategoryScale,
  LinearScale,
  PointElement,
  Title,
  Tooltip,
  Legend,
  annotationPlugin,
  ChartDataLabels
);

// Custom plugin to draw threshold line at y = 400
const drawThresholdLine = {
  id: "drawThresholdLine",
  beforeDraw(chart) {
    const { ctx, chartArea, scales } = chart;
    if (!chartArea) return;

    ctx.save();
    ctx.beginPath();
    ctx.setLineDash([6, 6]); // Dotted line style
    ctx.strokeStyle = "red";
    ctx.lineWidth = 2;

    const yPosition = scales.y.getPixelForValue(400);
    ctx.moveTo(chartArea.left, yPosition);
    ctx.lineTo(chartArea.right, yPosition);
    ctx.stroke();
    ctx.restore();
  },
};

const RealTimeGasChart = () => {
  const [gasData, setGasData] = useState([]);
  const [timestamps, setTimestamps] = useState([]);
  const [currentGasLevel, setCurrentGasLevel] = useState(0);

  useEffect(() => {
    const fetchData = async () => {
      try {
        const res = await fetch("/api/serial");
        const result = await res.json();
        const newValue = result.mq2;

        setGasData((prev) => [...prev.slice(-19), newValue]); // Keep last 20 values
        setTimestamps((prev) => [...prev.slice(-15), new Date().toLocaleTimeString()]);
        setCurrentGasLevel(newValue);
      } catch (error) {
        console.error("Error fetching data:", error);
      }
    };

    const interval = setInterval(fetchData, 100);
    return () => clearInterval(interval);
  }, []);

  // Dynamic background & border based on sensor value
  const isWarning = currentGasLevel > 400;
  const bgColor = isWarning ? "glass_red" : "glass";

  const data = {
    labels: timestamps,
    datasets: [
      {
        label: "Gas Sensor Data",
        data: gasData,
        borderColor: "rgba(54,162,235,1)",
        backgroundColor: "rgba(54,162,235,0.2)",
        borderWidth: 3,
        pointRadius: 4,
        pointBackgroundColor: "white",
        tension: 0.3, // Smooth curve
      },
    ],
  };

  const options = {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        labels: {
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
      },
      title: {
        display: true,
        text: "Gas Detection (MQ2 Sensor)",
        color: "#333",
        font: { size: 22, weight: "bold" },
      },
      datalabels: {
        display: true,
        color: "#000",
        font: { size: 12, weight: "bold" },
        formatter: (value) => value,
        anchor: "end",
        align: "top",
      },
    },
    scales: {
      x: {
        title: {
          display: true,
          text: "Time",
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
        ticks: { display: false },
        grid: { color: "rgba(0,0,0,0.1)" },
      },
      y: {
        title: {
          display: true,
          text: "Gas Sensor Value",
          color: "#333",
          font: { size: 14, weight: "bold" },
        },
        ticks: {
          color: "#333",
          stepSize: 150, // Decrease divisions by setting stepSize (adjust as needed)
        },
        grid: { color: "rgba(0,0,0,0.1)" },
        min: 0,
        max: 1050,
      },
    },
  };
  
  return (
    <div className={` flex flex-col items-center justify-center p-4 w-[99%] h-[99%] ${bgColor}`}>
      <Line data={data} options={options} plugins={[drawThresholdLine]} />
    </div>
  );
};

export default RealTimeGasChart;

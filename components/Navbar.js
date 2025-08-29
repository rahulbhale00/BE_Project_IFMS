"use client";

import { useState, useEffect } from "react";
import { Menu } from "lucide-react";

export default function Navbar({ onToggleSidebar }) {
  const [currentTime, setCurrentTime] = useState("");

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();
      const formatted = now.toLocaleTimeString([], {
        hour: "2-digit",
        minute: "2-digit",
        second: "2-digit",
      });
      setCurrentTime(formatted);
    };

    updateTime(); // Initial call
    const timer = setInterval(updateTime, 1000); // Update every second

    return () => clearInterval(timer); // Cleanup
  }, []);

  return (
    <>
      <nav className="bg-white glass m-2 shadow-md p-4 pr-6 flex items-center justify-between">
        <button onClick={onToggleSidebar} className="p-2 rounded-lg hover:bg-gray-200">
          <Menu size={24} />
        </button>
        <h1 className="text-xl font-semibold">Mini-Project</h1>
        <span className="text-xl font-medium text-gray-600">Current Time :- {currentTime}</span>
      </nav>
    </>
  );
}

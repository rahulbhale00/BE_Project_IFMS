import React from 'react';

const sensors = [
  { name: 'Flame Sensor', image: '/Flame_Sensor.jpg', info: [
      'Detects the presence of fire or flame.',
      'Sensitive to infrared radiation.',
      'Provides analog signal output.',
      'Used in fire detection and safety systems.',
      'Has a detection angle of about 60 degrees.',
      'Ideal for short range flame detection.'
    ] },
  { name: 'MQ2 Gas Sensor', image: '/MQ2-GasSensor.jpg', info: [
      'Detects LPG, smoke, methane, and other flammable gases.',
      'Offers analog output for gas concentration.',
      'Widely used in air quality monitoring.',
      'Low power consumption and cost-effective.',
      'Calibrated for quick gas leakage detection.',
      'Suitable for home safety and industrial applications.'
    ] },
  { name: 'Ultrasonic Sensor', image: '/ultasonic_sensor.jpg', info: [
      'Measures distance using ultrasonic waves.',
      'Non-contact distance measurement method.',
      'Accurate and reliable in various environments.',
      'Commonly used in robotics and obstacle detection.',
      'Operates with a transmitter and receiver module.',
      'Effective range typically from 2cm to several meters.'
    ] },
  { name: 'Vibrational Sensor', image: '/vibrationSensor.jpg', info: [
      'Detects vibrations and movements.',
      'Often based on piezoelectric technology.',
      'Used in security systems and equipment monitoring.',
      'Provides real-time data on mechanical disturbances.',
      'Can detect subtle changes in motion or resonance.',
      'Ideal for fault detection in machinery.'
    ] },
  { name: 'Temperature Sensor', image: '/Temp Sensor.jpg', info: [
      'Measures ambient temperature.',
      'Delivers accurate readings in real time.',
      'Can have analog or digital output.',
      'Utilized in HVAC, industrial, and consumer electronics.',
      'Some models have built-in calibration for precision.',
      'Often used in climate control and weather stations.'
    ] }
];

const Card = ({ sensor }) => (
  <div className="glass border-l-4 border-blue-500 p-4 rounded-lg shadow-lg bg-white flex flex-col items-center text-center h-full">
    {/* Sensor Name */}
    <h3 className="text-lg font-bold mb-4">{sensor.name}</h3>

    {/* Image (Fixed Size) */}
    <img src={sensor.image} alt={sensor.name} className="w-40 h-40 object-contain mb-4" />


    {/* Text Section (Fixed Height for Consistency) */}
    <div className="flex-1 flex flex-col justify-center">
      <ul className="list-none p-0 text-sm text-left">
        {sensor.info.map((point, idx) => (
          <li key={idx} className="relative pl-4 mb-1">&#9658; {point}</li>
        ))}
      </ul>
    </div>
  </div>
);


export default function Page() {
  return (
    <div className="h-[87vh] overflow-y-auto no-scrollbar pb-2">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-2">
        {sensors.map((sensor, idx) => (
          <Card key={idx} sensor={sensor} />
        ))}
      </div>
    </div>
  );
}

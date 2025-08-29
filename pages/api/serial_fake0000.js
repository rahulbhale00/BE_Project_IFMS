import { randomInt } from "crypto";

function getRandomVariation(value, percentage = 10) {
  const variation = Math.floor((value * percentage) / 100);
  return value + randomInt(-variation, variation + 1);
}

let fakeData = {
  out: 5,
  totalInside: 4,
  mq2: 450,
  flame: 700,
  vibration: 50,
  temperature: 25,
  in: 7 ,
};

export default function handler(req, res) {
  if (req.method === "GET") {
    const variedData = {
      out: getRandomVariation(fakeData.out),
      totalInside: getRandomVariation(fakeData.totalInside),
      mq2: getRandomVariation(fakeData.mq2),
      flame: getRandomVariation(fakeData.flame),
      vibration: getRandomVariation(fakeData.vibration),
      temperature: getRandomVariation(fakeData.temperature),
      in: getRandomVariation(fakeData.in),
    };
    res.status(200).json(variedData);
  } else {
    res.status(405).json({ error: "Method Not Allowed" });
  }
}

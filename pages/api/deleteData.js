import fs from 'fs';
import path from 'path';

export default function handler(req, res) {
  if (req.method !== 'DELETE') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const dataPath = path.join(process.cwd(), 'data_storage', 'data.json');
    const logsPath = path.join(process.cwd(), 'data_storage', 'logs.json');

    // Overwrite the files with empty arrays
    fs.writeFileSync(dataPath, JSON.stringify([], null, 2), 'utf-8');
    fs.writeFileSync(logsPath, JSON.stringify([], null, 2), 'utf-8');

    return res.status(200).json({ message: 'Data and logs successfully deleted!' });
  } catch (error) {
    console.error('Error deleting files:', error);
    return res.status(500).json({ error: 'Failed to delete data and logs' });
  }
}

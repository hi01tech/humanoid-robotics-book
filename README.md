# Physical AI & Humanoid Robotics Textbook

This repository contains the source code for the "Physical AI & Humanoid Robotics" textbook.

## Development

The project is split into two main components: a Docusaurus-based frontend and a Python backend.

### Frontend (Docusaurus)

To run the frontend development server:

```bash
cd frontend
npm install
npm start
```

### Server (Python)

To run the server:

```bash
cd server
pip install -r requirements.txt
uvicorn app.main:app --reload
```

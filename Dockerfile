FROM  python:3.11.11-bullseye

WORKDIR /usr/src/app

RUN apt-get update && apt-get install -y 

COPY requirements.txt ./
RUN pip install  --no-cache-dir -r requirements.txt

COPY ./files/devices.yaml ./files/
COPY ./modules/ ./modules/
COPY ./server/ ./server/
COPY ./api.py ./
COPY README.md ./

CMD ["python", "-m", "flask", "--app", "./api", "run", "--host=0.0.0.0", "--port=8080"]
# docker build -t planner-server .
# docker run --rm -it  --name planner -p 8004:8080  planner-server
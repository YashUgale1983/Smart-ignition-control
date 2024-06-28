const express = require("express");
const fileUpload = require("express-fileupload");
const fs = require("fs");
const { ImageAnnotatorClient } = require("@google-cloud/vision");
const cors = require("cors");
require("dotenv").config();

const app = express();
const port = 8888;
const filePath = "./resources/test.jpeg";

app.use(fileUpload());
app.use(express.json());
app.use(
  cors({
    origin: "*",
  })
);

const credential = JSON.parse(
  Buffer.from(process.env.GOOGLE_SERVICE_KEY, "base64").toString()
);

app.get("/", (req, res) => {
  res.status(200).json({
    msg: "successful get request",
  });
});

app.post("/detectLabels", async (req, res) => {
  console.log("request received : ", req);
  try {
    if (!req.files || !req.files.image) {
      return res.status(400).json({ error: "Image file is missing" });
    }

    const imageFile = req.files.image;
    const imageData = imageFile.data;

    fs.writeFileSync(filePath, imageData);

    const output = await labelAPI(filePath);
    if (output.result == "Helmet detected") {
      return res
        .status(200)
        .json({ message: "Helmet detected", labels: output.allLabels });
    } else {
      return res
        .status(200)
        .json({ message: "Helmet not detected", labels: output.allLabels });
    }
  } catch (error) {
    console.error("Error:", error);
    res.status(500).json({ error: "Internal Server Error" });
  }
});

async function labelAPI(imagePath) {
  // const client = new ImageAnnotatorClient({
  //   keyFilename:
  //     "/Users/yash/Projects/helmetdetection-420512-658d8e9b61aa.json",
  // });

  const client = new ImageAnnotatorClient({
    projectId: "helmetdetection-420512",
    // important
    credentials: {
      client_email: credential.client_email,
      private_key: credential.private_key,
    },
  });

  const [labelResult] = await client.labelDetection(imagePath);
  const [objectResult] = await client.objectLocalization(imagePath);

  const labelLabels = labelResult.labelAnnotations.map(
    (label) => label.description
  );
  const objectLabels = objectResult.localizedObjectAnnotations.map(
    (obj) => obj.name
  );

  const allLabels = labelLabels.concat(objectLabels);

  if (allLabels.includes("Helmet")) {
    // console.log("Helmet detected");
    return { result: "Helmet detected", allLabels };
  } else {
    // console.log("Helmet not detected");
    return { result: "Helmet not detected", allLabels };
  }
}

app.listen(port, () => {
  console.log(`Server is listening at port ${port}`);
});

// http://127.0.0.1:8888

// const fs = require("fs");
// const http = require("http");
// const server = http.createServer();
// const filePath = "./resources/test.jpeg";
// require("dotenv").config();

// server.on("request", (request, response) => {
//   if (request.method === "POST" && request.url === "/imageUpdate") {
//     const imageFile = fs.createWriteStream(filePath, { encoding: "utf8" });
//     request.on("data", function (data) {
//       imageFile.write(data);
//     });

//     request.on("end", async function () {
//       imageFile.end();
//       const labels = await labelAPI();
//       console.log(labels);
//       response.writeHead(200, { "Content-Type": "application/json" });
//       response.end(JSON.stringify(labels));
//     });
//   } else if (request.method === "GET" && request.url === "/test") {
//     response.writeHead(200, { "Content-Type": "text/plain" });
//     response.end("Test successful");
//   } else {
//     console.log("error");
//     response.writeHead(405, { "Content-Type": "text/plain" });
//     response.end();
//   }
// });

// async function labelAPI() {
//   const o = [];
//   // Imports the Google Cloud client library
//   const { ImageAnnotatorClient } = require("@google-cloud/vision");

//   const credential = JSON.parse(
//     Buffer.from(process.env.GOOGLE_SERVICE_KEY, "base64").toString()
//   );

//   // Creates a client
//   const client = new ImageAnnotatorClient({
//     projectId: "helmetdetection-420512",
//     // important
//     credentials: {
//       client_email: credential.client_email,
//       private_key: credential.private_key,
//     },
//   });

//   // Performs label detection on the image file
//   const [result] = await client.labelDetection(filePath);
//   console.log(result);
//   const labels = result.labelAnnotations;

//   labels.forEach((label) => {
//     o.push({ description: label.description, score: label.score });
//   });
//   return o;
// }

// const port = 8888;
// server.listen(port);
// console.log(`Listening at ${port}`);

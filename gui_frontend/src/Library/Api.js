import axios from 'axios'
import config from '../config.json'

async function getRequest(request_url) {
    const response = await axios.get(request_url).catch(function (error) {
      console.log(error.response.status)
      console.log(error.response.data)
      return error.response
    })
    console.log(response.data)
    return response.data
}

export class HttpError extends Error {
  constructor(msg) {
    super(`HttpError: ${msg}`)
  }
}

export async function Api(requestType, controller, url, payload) {
    let request_url = ""
    switch(requestType) {
        default:
            break
        case "GET":
            request_url = config.BACKEND_URL + controller + '/' + url
            console.log("Sending Request: "+ request_url + " Request-Type: " + requestType)
              return getRequest(request_url).then(data => data)
        case "POST":
            request_url = config.BACKEND_URL + controller + '/' + url
            console.log("Sending Request: "+ request_url + " Request-Type: " + requestType)
              const response = await axios.post(request_url, payload, {
                headers: {'Content-Type': 'application/json'}
            }).catch(function (error) {
              console.log(error.response.status)
              console.log(error.response.data)
              return error.response
            })
            console.log(response)
            return response
    }
}

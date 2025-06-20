import { Injectable } from '@angular/core';
import { Observable, Subject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class GlidernetService {
  private ws: WebSocket | null = null;
  private messages = new Subject<any>();

  constructor() {
    this.connect();
  }

  private connect(): void {
    this.ws = new WebSocket('ws://localhost:8080');
    this.ws.onopen = () => {
      console.log('WebSocket connected');
    };
    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.message) {
        const parsed = this.parseAprsMessage(data.message);
        if (parsed) {
          this.messages.next(parsed);
        }
      }
    };
    this.ws.onerror = (err) => {
      console.error('WebSocket error:', err);
      setTimeout(() => this.connect(), 5000);
    };
    this.ws.onclose = () => {
      console.log('WebSocket closed');
      setTimeout(() => this.connect(), 5000);
    };
  }

  public getMessages(): Observable<any> {
    return this.messages.asObservable();
  }

  private parseAprsMessage(message: string): any {
    // Parse OGN messages (OGADSB, OGFLR, OGNFNT, OGNSDR)
    const match = message.match(/([^>]+)>(OG[A-Z]+),[^:]+:\/(\d{6}h)(\d{4}\.\d{2}N)(?:I)?\S*(\d{5}\.\d{2}E)\S*(?:(?:(\d{1,3})\/(\d{1,3}))?).*?(?:\/A=(\d{6})|g(\d+))?.*?(?:!W\d{2}!)?\s*(?:id([0-9A-F]+))?\s*(?:([+-]\d+fpm))?.*?(?:A3:([^\s]+))?.*?(?:Sq(\d{4}))?.*?/);
    if (match) {
      const [, name, protocol, time, lat, lon, course, speed, altA, altG, id, climb, flightNumber, squawk] = match;
      const latitude = this.parseCoordinate(lat);
      const longitude = this.parseCoordinate(lon);
      const altitude = altA ? parseInt(altA, 10) / 3.281 : altG ? parseInt(altG, 10) : 0; // Feet to meters or ground altitude
      return {
        name: flightNumber || name,
        protocol,
        time,
        latitude,
        longitude,
        altitude,
        course: course ? parseInt(course, 10) : 0,
        speed: speed ? parseInt(speed, 10) : 0,
        id: id || null,
        climb: climb || '0fpm',
        flightNumber: flightNumber || null,
        squawk: squawk || null
      };
    } else {
      console.log(`unknown: ${message}`);
    }
    return null;
  }

  private parseCoordinate(coord: string): number {
    const match = coord.match(/(\d{2,3})(\d{2}\.\d{2})(N|S|E|W)/);
    if (match) {
      const degrees = parseInt(match[1], 10);
      const minutes = parseFloat(match[2]);
      const direction = match[3] === 'N' || match[3] === 'E' ? 1 : -1;
      return (degrees + minutes / 60) * direction;
    }
    return 0;
  }
}

import { Component, AfterViewInit } from '@angular/core';
import * as L from 'leaflet';

import { GlidernetService } from '../services/glidernet-service';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-viewer',
  imports: [],
  templateUrl: './viewer.html',
  styleUrl: './viewer.scss',
  standalone: true, // Ensure standalone if using standalone components
})
export class Viewer implements AfterViewInit {
  private map: L.Map | undefined;
  private aircraftLayer: L.LayerGroup = L.layerGroup();
  private aircraftMap: Map<
    string,
    {
      marker: L.Marker;
      polyline: L.Polyline;
      positions: [number, number][];
      timeout: number;
      data: any;
    }
  > = new Map();
  private subscription: Subscription | undefined;

  constructor(
    private glidernetService: GlidernetService,
  ) {
  }

  ngAfterViewInit(): void {
    this.initMap();
  }

  private initMap(): void {
    // The lib looks for the icons in the media directory.
    const iconRetinaUrl = 'marker-icon-2x.png';
    const iconUrl = 'marker-icon.png';
    const shadowUrl = 'marker-shadow.png';
    L.Icon.Default.mergeOptions({
      iconRetinaUrl,
      iconUrl,
      shadowUrl
    });

    const markerIconAirportMajor = L.icon({
      iconUrl: 'media/marker-icon-airport-major.png',
      iconSize: [24, 36],
      iconAnchor: [16, 16],
      popupAnchor: [0, -16],
      shadowUrl: 'media/marker-shadow.png',
      shadowSize: [41, 41]
    });

    const markerIconAirportRegional = L.icon({
      iconUrl: 'media/marker-icon-airport-regional.png',
      iconSize: [20, 30],
      iconAnchor: [14, 14],
      popupAnchor: [0, -14],
      shadowUrl: 'media/marker-shadow.png',
      shadowSize: [41, 41]
    });

    const markerIconAirportTraining = L.icon({
      iconUrl: 'media/marker-icon-airport-training.png',
      iconSize: [16, 24],
      iconAnchor: [12, 12],
      popupAnchor: [0, -12],
      shadowUrl: 'media/marker-shadow.png',
      shadowSize: [41, 41]
    });

    const aircraftIcon = L.icon({
      iconUrl: 'media/aircraft-icon.png',
      iconSize: [20, 20],
      iconAnchor: [10, 10],
      popupAnchor: [0, -10],
    });

    const southWest = L.latLng(45.8179, 5.9559);
    const northEast = L.latLng(47.8084, 10.4923);
    const bounds = L.latLngBounds(southWest, northEast);
    this.map = L.map('viewer', {
      center: [46.8182, 8.2275], // Center of Switzerland
      zoom: 9,
      minZoom: 9,
      maxBounds: bounds,
      maxBoundsViscosity: 1.0, // 1.0 = no panning outside
    });


    L.tileLayer('https://tiles.stadiamaps.com/tiles/stamen_terrain/{z}/{x}/{y}{r}.png', {
      attribution: '&copy; <a href="https://www.stadiamaps.com/" target="_blank">Stadia Maps</a> &copy; <a href="https://www.stamen.com/" target="_blank">Stamen Design</a> &copy; <a href="https://openmaptiles.org/" target="_blank">OpenMapTiles</a> &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      subdomains: 'abcd',
      maxZoom: 20
    }).addTo(this.map);

    /*
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(this.map);
    */

    const airports = [
      { name: 'Zurich (ZRH)', coords: { lat: 47.4647, lng: 8.5492 }, type: 'major' },
      { name: 'Geneva (GVA)', coords: { lat: 46.2381, lng: 6.1089 }, type: 'major' },
      { name: 'Basel-Mulhouse (BSL)', coords: { lat: 47.5896, lng: 7.5299 }, type: 'major' },
      { name: 'Bern (BRN)', coords: { lat: 46.9141, lng: 7.4971 }, type: 'regional' },
      { name: 'Lugano (LUG)', coords: { lat: 46.0043, lng: 8.9106 }, type: 'regional' },
      { name: 'St. Gallen-Altenrhein (ACH)', coords: { lat: 47.4850, lng: 9.5608 }, type: 'regional' },
      { name: 'Sion (SIR)', coords: { lat: 46.2196, lng: 7.3268 }, type: 'regional' },
      { name: 'Samedan (SMV)', coords: { lat: 46.5341, lng: 9.8841 }, type: 'regional' },
      { name: 'Grenchen (ZHI)', coords: { lat: 47.1816, lng: 7.4172 }, type: 'regional' },
      { name: 'Lausanne-La Blécherette (QLS)', coords: { lat: 46.5453, lng: 6.6167 }, type: 'regional' },
      { name: 'Birrfeld (LSZF)', coords: { lat: 47.4436, lng: 8.2344 }, type: 'training' },
      { name: 'Buttwil (LSZU)', coords: { lat: 47.3269, lng: 8.2956 }, type: 'training' },
      { name: 'Mollis (LSZM)', coords: { lat: 47.0789, lng: 9.0648 }, type: 'training' },
      { name: 'Triengen (LSPN)', coords: { lat: 47.2267, lng: 8.0781 }, type: 'training' },
      { name: 'Les Eplatures (LSGC)', coords: { lat: 47.0839, lng: 6.7931 }, type: 'training' }
    ];

    // Add airport markers
    airports.forEach(airport => {
      const icon = airport.type === 'major' ? markerIconAirportMajor
        : airport.type === 'regional' ? markerIconAirportRegional
        : markerIconAirportTraining;
      L.marker(airport.coords, { icon })
        .addTo(this.map!)
        .bindPopup(`<b>${airport.name}</b>`);
    });

    this.aircraftLayer.addTo(this.map!);

    // Process incoming OGN messages.
    this.subscription = this.glidernetService.getMessages().subscribe((data: any) => {
      if (data.latitude && data.longitude && data.name) {
        const aircraftId = data.name; // Unique identifier (e.g., DLH89T, FLR200C68)

        // Update or create aircraft entry
        const existing = this.aircraftMap.get(aircraftId);
        if (existing) {
          clearTimeout(existing.timeout);
          existing.marker.setLatLng([data.latitude, data.longitude]);
          existing.data = data;
          existing.positions.push([data.latitude, data.longitude]);
          if (existing.positions.length > 100) {
            existing.positions.shift(); // Limit the size of the array
          }
          existing.polyline.setLatLngs(existing.positions);
        } else {
          const popupContent = this.getPopupContent(data);
          const marker = L.marker([data.latitude, data.longitude], { icon: aircraftIcon })
            .bindPopup(popupContent);
          const polyline = L.polyline([[data.latitude, data.longitude]], {
            color: 'black',
            weight: 3,
            opacity: 0.7
          });
          marker.addTo(this.aircraftLayer);
          polyline.addTo(this.aircraftLayer);
          this.aircraftMap.set(aircraftId, {
            marker,
            polyline,
            positions: [[data.latitude, data.longitude]],
            timeout: null as any,
            data
          });
        }

        // Set new timeout to remove marker and polyline after a while.
        const timeout = setTimeout(() => {
          const aircraft = this.aircraftMap.get(aircraftId);
          if (aircraft) {
            this.aircraftLayer.removeLayer(aircraft.marker);
            this.aircraftLayer.removeLayer(aircraft.polyline);
            this.aircraftMap.delete(aircraftId);
          }
        }, 60000); // 8000);
        this.aircraftMap.get(aircraftId)!.timeout = timeout;

        // Update popup content
        this.aircraftMap.get(aircraftId)!.marker.setPopupContent(this.getPopupContent(data));
      }
    });
  }

  ngOnDestroy(): void {
    // Clean up subscription and timeouts
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
    this.aircraftMap.forEach(({ timeout, marker, polyline }) => {
      clearTimeout(timeout);
      this.aircraftLayer.removeLayer(marker);
      this.aircraftLayer.removeLayer(polyline);
    });
    this.aircraftMap.clear();
  }

  private getPopupContent(data: any): string {
    return `
      <b>${data.name}</b><br>
      Protocol: ${data.protocol}<br>
      Altitude: ${data.altitude.toFixed(0)} m<br>
      Climb: ${data.climb}<br>
      Course: ${data.course}°<br>
      Speed: ${data.speed} knots
      ${data.flightNumber ? `<br>Flight: ${data.flightNumber}` : ''}
      ${data.squawk ? `<br>Squawk: ${data.squawk}` : ''}
    `;
  }
}

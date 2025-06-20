import { Routes } from '@angular/router';
import { Viewer } from './viewer/viewer';

export const routes: Routes = [
  { path: '', redirectTo: '/viewer', pathMatch: 'full' }, // Default route
  { path: 'viewer', component: Viewer } // Map route
];

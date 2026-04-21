## Fasi del progetto

### 1. Studio preliminare del setup
Prima fase di familiarizzazione con il banco prova, la catena di misura e la documentazione tecnica.

Attività principali:
- studio di slide, datasheet, manuali e template Simulink;
- comprensione di ingressi, uscite e grandezze fisiche del sistema;
- identificazione dei sottosistemi principali: bobina, amplificatore, sensore di posizione, misura di corrente.

### 2. Modellazione del sistema
Costruzione del modello del magnetic levitation system sia in forma non lineare sia linearizzata attorno al punto di equilibrio.

Attività principali:
- derivazione del modello non lineare del sistema elettromeccanico;
- linearizzazione nel punto di lavoro;
- costruzione della rappresentazione in spazio di stato;
- ricavo della funzione di trasferimento utile all'analisi FB.

Output atteso:
- modello non lineare;
- modello linearizzato;
- modello state-space;
- funzione di trasferimento G(s).

### 3. Identificazione dei parametri incerti
Stima dei parametri non direttamente misurabili o poco affidabili, così da rendere il modello coerente con il banco reale.

Attività principali:
- identificazione dei parametri elettrici, meccanici e di conversione;
- stima di costanti di sensore, guadagni equivalenti e altri parametri incerti;
- aggiornamento del modello con i valori identificati.

### 4. Validazione temporale del modello
Confronto tra il comportamento del modello e la risposta reale nel dominio del tempo.

Attività principali:
- applicazione di un ingresso impulso o gradino a **u(t)**, oppure a **y°(t)** in anello chiuso;
- acquisizione della risposta sperimentale **y(t)** e, se utile, anche **u(t)**;
- confronto con la risposta simulata del modello;
- valutazione di guadagno, tempo di assestamento, overshoot e forma della risposta.

### 5. Validazione in frequenza del modello
Verifica della coerenza tra modello e sistema reale nel dominio della frequenza.

Attività principali:
- applicazione di ingressi sinusoidali di frequenza diversa a **u(t)**, oppure a **y°(t)** in anello chiuso;
- misura delle risposte sinusoidali **y(t)** e, se utile, **u(t)**;
- ricostruzione della risposta in frequenza sperimentale;
- confronto con il diagramma di Bode del modello.

### 6. Analisi in anello aperto
Studio delle proprietà del sistema prima della sintesi dei controllori.

Attività principali:
- analisi della stabilità open-loop;
- verifica di controllabilità e osservabilità del modello linearizzato;
- valutazione del range operativo e dei vincoli fisici;
- analisi preliminare delle prestazioni e delle criticità del setup.

### 7. Definizione delle specifiche di controllo
Definizione di un insieme di specifiche ragionate per ciascun task di controllo.

Attività principali:
- definizione di obiettivi su rapidità, overshoot, precisione e robustezza;
- definizione dei vincoli su tensione, corrente, posizione e sicurezza;
- organizzazione delle specifiche per task FB, SS e AC.

### 8. Frequency-Based Control, EM coil current control
Sviluppo del controllo di corrente della bobina con tecniche frequency-based.

Attività principali:
- utilizzo della funzione di trasferimento del sottosistema di corrente;
- progetto del controllore **R(s)** in modo che l'anello aperto **L(s)** abbia le proprietà desiderate;
- sintesi tramite **Bode**, **Root Locus** o **Nyquist**;
- verifica in simulazione di banda, margini e risposta temporale.

Output atteso:
- controllore FB per il current loop.

### 9. Frequency-Based Control, ball position stabilization
Primo approccio al controllo di posizione della pallina tramite metodi FB.

Attività principali:
- utilizzo del modello linearizzato posizione-comando o posizione-corrente;
- progetto di un controllore stabilizzante per l'equilibrio instabile;
- verifica dei margini di stabilità e delle prestazioni in simulazione.

Output atteso:
- controllore FB per la stabilizzazione della posizione.

### 10. State-Space Control, Pole Placement + Luenberger Observer
Approccio state-space al problema di stabilizzazione della pallina.

Attività principali:
- utilizzo del modello linearizzato in spazio di stato;
- progetto del feedback di stato tramite **Pole Placement**;
- progetto dell'osservatore di **Luenberger** per stimare gli stati non misurati;
- verifica in simulazione della dinamica in anello chiuso.

Output atteso:
- controllore SS con Pole Placement e osservatore di Luenberger.

### 11. State-Space Control, Linear Quadratic + Kalman Filter
Seconda strategia state-space, più orientata a prestazioni e stima ottima.

Attività principali:
- progetto del regolatore **LQR/LQ**;
- scelta e taratura delle matrici di peso;
- progetto del **Kalman Filter** come osservatore/stimatore;
- confronto con il Pole Placement in termini di prestazioni e robustezza.

Output atteso:
- controllore SS con LQ e Kalman Filter.

### 12. Trajectory tracking e lift-up, SS e Advanced Control
Sviluppo del terzo task, cioè inseguimento di traiettoria 1D e lift-up della pallina.

Attività principali:
- estensione del controllore state-space al tracking di riferimento;
- eventuale introduzione di azione integrale o struttura servo;
- sviluppo di una strategia di **Advanced Control**, per esempio MPC, Sliding Mode, Feedback Linearization oppure controllo robusto/adattativo;
- gestione della fase di lift-up e della successiva stabilizzazione/tracking.

Output atteso:
- strategia completa per tracking e lift-up.

### 13. Validazione dei controllori
Ogni controllore progettato deve essere validato sul modello e, quando possibile, sul sistema reale.

Attività principali:
- test in simulazione;
- test sperimentali sul banco;
- validazione nel dominio del tempo tramite gradino e impulso;
- validazione nel dominio della frequenza tramite ingressi sinusoidali e confronto con i Bode;
- confronto tra risposta del modello e risposta reale in anello chiuso.

### 14. Analisi critica e confronto finale
Fase conclusiva in cui si confrontano tutte le strategie sviluppate.

Attività principali:
- confronto tra approcci FB, SS e AC;
- valutazione di prestazioni, robustezza e complessità implementativa;
- discussione dei vantaggi e dei limiti di ciascun controllore;
- selezione delle figure e dei risultati finali per il report.
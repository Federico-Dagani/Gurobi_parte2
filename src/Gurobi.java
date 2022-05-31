import gurobi.*;
import gurobi.GRB.IntParam;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Gurobi {

    private static final int N_VERTICI = 43; //costante corrispondente al numero di vertici o nodi del problema
    private static final int M = 200; //costante M abbastanza stringente utilizzata per applicare un vincolo disgiuntivo nel punto C del quesito 3

    public static void main(String[] args) throws IOException {
        //lettura file txt
        int v = 34;
        int a = 9;
        int b1 = 13;
        int b2 = 25;
        int c = 138;
        int d1 = 0;
        int d2 = 14;
        int e1 = 27;
        int e2 = 24;
        int f1 = 38;
        int f2 = 39;
        int g1 = 40;
        int g2 = 26;
        int h1 = 33;
        int h2 = 32;
        int i1  = 10;
        int i2 = 7;
        int l = 7;

        String file_name = "src/coppia16.txt";
        //lettura tramite parsing dei rimanenti dati del file
        int[][] costi = new int[N_VERTICI][N_VERTICI];
        parsing_file(file_name, costi);

        try{

            GRBEnv env = new GRBEnv("Coppia16.log");
            env.set(IntParam.Presolve, 0);
            env.set(IntParam.Method, 0);


            //---------------------------------------QUESITO I---------------------------------------------------

            //modello 1 gurobi
            GRBModel model = new GRBModel(env);

            //creazione variabili Xij
            GRBVar[][] Xij = creazione_Xij(model);

            //creazione variabili u di supporto
            GRBVar[] u = creazione_u(model);

            //---------------------------------------F.O.-------------------------------------

            f_o(model, Xij, costi);

            //-------------------------------------VINCOLI------------------------------------

            vincoli_assegnamento(model, Xij);

            vincoli_u(model, Xij, u);

            //ottimizzazione modello gurobi
            model.optimize();

            //salvataggio e display del valore della funzione obiettivo
            double funzione_obiettivo_1 = model.get(GRB.DoubleAttr.ObjVal);

            //calcolo ciclo modello 1
            ArrayList<Integer> ciclo1 = calcola_ciclo(Xij);

            //---------------------------------------QUESITO II--------------------------------------------------

            //creo un nuovo modello identico al primo e aggiungo il vincolo che impone il raggiungimento
            //dello stesso risultato della funzione obiettivo del quesito 1

            GRBModel model2 = new GRBModel(env);
            //creazione variabili
            Xij = creazione_Xij(model2);
            u = creazione_u(model2);

            //---------------------------------------F.O.-------------------------------------

            f_o(model2, Xij, costi);

            //-------------------------------------VINCOLI------------------------------------

            vincoli_assegnamento(model2, Xij);

            vincoli_u(model2, Xij, u);

            //Vincolo aggiunto rispetto al quesito I.
            //Impone che la funzione obiettivo sia pari a quella del primo quesito
            GRBLinExpr expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model2.addConstr(expr, GRB.EQUAL, funzione_obiettivo_1, "valori funzioni obiettivo 1 e 2 uguali");

            //ottimizzazione
            model2.optimize();

            //calcolo ciclo 2
            ArrayList<Integer> ciclo2 = calcola_ciclo(Xij);



            //---------------------------------------QUESITO III-------------------------------------------------

            GRBModel model3 = new GRBModel(env);
            //creazione variabili
            Xij = creazione_Xij(model3);
            u = creazione_u(model3);
            //creazione variabili z implementate nella nuova funzione obiettivo per implementare il punto D del quesito 3 del compito
            GRBVar[] z = creazione_z(model3);

            //---------------------------------------F.O.-------------------------------------

            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            //aggiungo la variabile binaria z[0] alla f.o. moltiplicata per l; se z[0] è pari a 1, allora la f.o. varrà l di più del dovuto (punto d)
            expr.addTerm(l, z[0]);

            model3.setObjective(expr);
            model3.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //-------------------------------------VINCOLI------------------------------------

            //------------------A-------------------
            //il lato (d1, d2) sia percorribile se e solo se sono percorsi anche i lati (e1, e2) e (f1, f2)
            //2*Xij[d] - Xij[e] - Xij[f] <= 0
            expr = new GRBLinExpr();
            expr.addTerm(2,Xij[d1][d2]);  //bisogna considerare sia il link dal nodo d1 a d2 sia il link contrario dal nodo d2 a d1
            expr.addTerm(2, Xij[d2][d1]); //così per ogni vincolo aggiuntivo che segue nel medesimo quesito
            expr.addTerm(-1,Xij[f1][f2]);
            expr.addTerm(-1,Xij[f2][f1]);
            expr.addTerm(-1,Xij[e1][e2]);
            expr.addTerm(-1,Xij[e2][e1]);
            model3.addConstr(expr, GRB.LESS_EQUAL, 0, "il lato (d1, d2) sia percorribile se e solo se sono percorsi anche i lati (e1, e2) e (f1, f2)");

            //------------------B-------------------
            //il costo dei lati incidenti a v sia al massimo il a% del costo totale del ciclo
            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++){
                expr.addTerm(costi[i][v], Xij[i][v]);
            }
            for(int j=0; j< N_VERTICI; j++){
                expr.addTerm(costi[v][j], Xij[v][j]);
            }
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(-costi[i][j]*a/100, Xij[i][j]);
                }
            }
            model3.addConstr(expr, GRB.LESS_EQUAL, 0 , "il costo dei lati incidenti a v sia al massimo il a% del costo");

            //------------------C-------------------
            //se il lato (b1,b2) viene percorso, il costo del ciclo ottimo sia inferiore a c
            //utilizzo il valore M abbastanza stringente per creare un vincolo disgiuntivo
            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            expr.addTerm(-c, Xij[b1][b2]);
            expr.addTerm(-c, Xij[b2][b1]);
            expr.addTerm(M, Xij[b1][b2]);
            expr.addTerm(M, Xij[b2][b1]);
            model3.addConstr(expr, GRB.LESS_EQUAL, M,"se il lato b1 b2 viene percorso, il costo del ciclo sia inferiore a c");

            //------------------D-------------------
            //nel caso in cui i lati (g1, g2), (h1, h2) e (i1, i2) vengano tutti percorsi, si debba pagare un costo aggiuntivo pari a l.
            //utilizzo le variabili binarie z[i]: Xij[g] + Xij[h] + Xij[i] = 3z[0] + 2z[1] + 1z[2]
            //quando 3 di questi link vengono attivati allora z[0] deve valere 1. (oppure z[1] e z[2] contemporaneamente, ma lo risolvo al prossimo vincolo)
            expr = new GRBLinExpr();
            expr.addTerm(1,Xij[g1][g2]);
            expr.addTerm(1,Xij[g2][g1]);
            expr.addTerm(1,Xij[h1][h2]);
            expr.addTerm(1,Xij[h2][h1]);
            expr.addTerm(1,Xij[i1][i2]);
            expr.addTerm(1,Xij[i2][i1]);
            expr.addTerm(-3, z[0]);
            expr.addTerm(-2, z[1]);
            expr.addTerm(-1, z[2]);
            model3.addConstr(expr, GRB.EQUAL, 0, "se i lat1 g1-g2, h1-h2, i1-i2  viengono percorsi, il costo del ciclo aumenta di l");
            //imposto che al più una sola z sia pari a 1 cosicchè non sia possibile avere z[1] e z[2] attive contemporaneamente.
            expr = new GRBLinExpr();
            for(int k=0; k<3; k++){
                expr.addTerm(1,z[k]);
            }
            model3.addConstr(expr, GRB.LESS_EQUAL, 1, "al più solo uno z attivo");

            vincoli_assegnamento(model3, Xij);

            vincoli_u(model3, Xij, u);

            //ottimizzazione
            model3.optimize();

            //salvataggio e display del valore della funzione obiettivo
            double funzione_obiettivo_3 = model3.get(GRB.DoubleAttr.ObjVal);

            //calcolo ciclo 3
            ArrayList<Integer> ciclo3 = calcola_ciclo(Xij);

            stampa_finale((int)funzione_obiettivo_1, ciclo1, ciclo2, (int)funzione_obiettivo_3, ciclo3);

        }catch(GRBException e){
            System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
    }

    /**
     * metodo di lettura della matrice costi tra i vari nodi. Il salvataggio degli interi avviene dopo la lettura della stringa "Vertici ".
     * @param file_name nome del file da cui leggere i dati
     * @param costi matrice inizialmente vuota che viene riempita durante la lettura del file con i dati contenuti nel file
     * @throws IOException
     */
    public static void parsing_file(String file_name, int[][] costi) throws IOException {

        File file = new File(file_name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        String lettura;
        boolean start_parse = false;

        String parola_start_read = "Vertici " + N_VERTICI;

        while((lettura = br.readLine()) != null){
            if(start_parse){
                int[] numeri_letti = Arrays.stream(lettura.split(" ")).mapToInt(Integer::parseInt).toArray();
                costi[numeri_letti[0]][numeri_letti[1]] = costi[numeri_letti[1]][numeri_letti[0]]= numeri_letti[2];
            }
            if(lettura.equals(parola_start_read)){
                start_parse = true;
            }
        }
        //imposto la diagonale della matrice pari a 0; il costo da un nodo a sè stesso è pari a 0.
        for(int i= 0; i < N_VERTICI; i++){
            costi[i][i] = 0;
        }
    }

    /**
     * metodo che gestisce la stampa a video dei risultati del compito
     * @param funzione_obiettivo_1 ottimo del modello 1
     * @param ciclo1 ciclo ottimo del modello 1
     * @param ciclo2 ciclo ottimo del modello 2
     * @param funzione_obiettivo_3 ottimo del modello 3
     * @param ciclo3 ciclo ottimo del modello 3
     */
    public static void stampa_finale(int funzione_obiettivo_1, ArrayList<Integer> ciclo1, ArrayList<Integer> ciclo2, int funzione_obiettivo_3, ArrayList<Integer> ciclo3){
        //---------------------STAMPA A VIDEO-------------------------
        System.out.printf("\n\n\n");
        System.out.println("GRUPPO <coppia 16>");
        System.out.println("Componenti: <Bresciani Simone> <Dagani Federico>\n");
        System.out.println("QUESITO I:");
        System.out.printf("funzione obiettivo = %d\n", funzione_obiettivo_1);
        System.out.print("ciclo ottimo 1: ");
        System.out.println(ciclo1);
        System.out.println("\nQUESITO II:");
        System.out.print("ciclo ottimo 2: ");
        System.out.println(ciclo2);
        System.out.println("\nQUESITO III:");
        System.out.printf("funzione obiettivo = %d\n", funzione_obiettivo_3);
        System.out.print("ciclo ottimo 3: ");
        System.out.println(ciclo3);
    }

    /**
     * metodo che controlla quali Xij sono poste pari ad 1 e costruisce il ciclo passando link per link
     * @param Xij variabili binarie Xij
     * @return un array di lunghezza N_VERTICI+1 che identifica il ciclo fatto che parte dal nodo 0 e termine al nodo 0.
     * @throws GRBException
     */
    public static ArrayList<Integer> calcola_ciclo(GRBVar[][] Xij) throws GRBException {
        ArrayList<Integer> ciclo = new ArrayList<>();
        int precedente=0;
        ciclo.add(precedente);
        for(int i=0; i<N_VERTICI; i++){
            for(int k= 0; k< N_VERTICI; k++) {
                if (Xij[precedente][k].get(GRB.DoubleAttr.X) == 1) {
                    ciclo.add(k);
                    precedente=k;
                    break;
                }
            }
        }
        return ciclo;
    }

    /**
     * metodo che impone al modello di gurobi i vincoli u affinchè la soluzione trovata non abbia cicli multipli al suo interno
     * @param model modello gurobi
     * @param Xij variabili binarie Xij
     * @param u variabili di supporto u
     * @throws GRBException
     */
    public static void vincoli_u(GRBModel model, GRBVar[][] Xij, GRBVar[] u) throws GRBException{

        //vincoli di Miller-Tucker-Zemil utilizzando le variabili u di supporto
        //u[0] = 1
        GRBLinExpr expr = new GRBLinExpr();
        expr.addTerm(1,u[0]);
        model.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
        //2 <= u[i] <= N con i= 1...N-1
        for (int i=1; i<N_VERTICI; i++){
            expr = new GRBLinExpr();
            expr.addTerm(1,u[i]);
            model.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
            expr = new GRBLinExpr();
            expr.addTerm(1,u[i]);
            model.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + (N_VERTICI-1));
        }
        //u[j] >= u[i] + 1 - ((N-1)-1)(1-xij) con  i != j  e  i,j = 1...N-1
        //svolgendo il calcolo: u[j] >= u[i] + 2 - (N-1) + xij((N-1)-1)
        for(int i=1; i<N_VERTICI-1; i++){
            for(int j=1; j<N_VERTICI; j++){
                expr = new GRBLinExpr();
                if(i!=j) {
                    expr.addTerm((N_VERTICI - 1) - 1 , Xij[i][j]);
                    expr.addTerm(-1, u[j]);
                    expr.addTerm(1, u[i]);
                    model.addConstr(expr, GRB.LESS_EQUAL, (N_VERTICI- 1) - 2 , "vincolo di sequenzialità delle variabili u");
                }
            }
        }
    }

    /**
     * metodo che impone al modello di gurobi i vincoli di assegnamento affinchè per ogni nodo ci sia un solo vincolo di entrata
     * e uno di uscita e che questi siano diversi tra loro
     * @param model modello di gurobi
     * @param Xij variabili binare Xij
     * @throws GRBException
     */
    public static void vincoli_assegnamento(GRBModel model, GRBVar[][] Xij) throws GRBException{

        GRBLinExpr expr;
        //vincoli di assegnamento:
        //una sola uscita attiva per ogni vertice
        for(int i= 0; i< N_VERTICI; i++){
            expr = new GRBLinExpr();
            for(int j=0; j<N_VERTICI; j++){
                expr.addTerm(1, Xij[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 1, "una sola uscita per ogni vertice");
        }
        //una sola entrata attiva per ogni vertice
        for(int i= 0; i< N_VERTICI; i++){
            expr = new GRBLinExpr();
            for(int j=0; j<N_VERTICI; j++){
                expr.addTerm(1, Xij[j][i]);
            }
            model.addConstr(expr, GRB.EQUAL, 1, "una sola entrata per ogni vertice");
        }

        //aggiungo N vincoli che impongano che il ciclo non si fermi in un nodo
        for(int i= 0; i< N_VERTICI; i++){
            expr = new GRBLinExpr();
            expr.addTerm(1, Xij[i][i]);
            model.addConstr(expr, GRB.EQUAL, 0, "diagonale nulla");
        }

        //aggiungo N*(N-1)/2 vincoli che impongano che il verso entrante e il verso uscente per ogni Xij non sia il medesimo
        //scorro la triangolare superiore della matrice
        for(int i = 0; i< N_VERTICI-1; i++){
            for(int j = i+1; j<N_VERTICI; j++){
                expr = new GRBLinExpr();
                expr.addTerm(1, Xij[i][j]);
                expr.addTerm(1, Xij[j][i]);
                model.addConstr(expr, GRB.LESS_EQUAL, 1, "al più un collegamento attivo tra due nodi(non entrambi andata e ritorno)");
            }
        }
    }

    /**
     * metodo che impone al modello di gurobi la funzione obiettivo del problema: minimizzare il costo totale del ciclo
     * @param model modello gurobi
     * @param Xij variabili binarie Xij
     * @param costi matrice definita contenente i costi per ciascun link
     * @throws GRBException
     */
    public static void f_o(GRBModel model, GRBVar[][] Xij, int[][] costi) throws GRBException{
        GRBLinExpr expr = new GRBLinExpr();
        for(int i=0; i< N_VERTICI; i++) {
            for (int j = 0; j < N_VERTICI; j++) {
                expr.addTerm(costi[i][j], Xij[i][j]);
            }
        }
        model.setObjective(expr);
        model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);
    }

    /**
     * metodo che dato un modello gurobi crea al suo interno una matrice di variabili binarie Xij
     * @param model modello gurobi
     * @return matrice quadrata di variabili binarie Xij di grandezza N_VERTICI
     * @throws GRBException
     */
    public static GRBVar[][] creazione_Xij(GRBModel model) throws GRBException{

        GRBVar[][] Xij = new GRBVar[N_VERTICI][N_VERTICI];
        for(int i=0; i< N_VERTICI; i++){
            for(int j=0; j< N_VERTICI; j++) {
                Xij[i][j] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + i + "_" + j);
            }
        }
        return Xij;
    }

    /**
     * metodo che dato un modello gurobi crea al suo interno un vettore di variabili intere u
     * @param model modello gurobi
     * @return vettore di variabili intere u di grandezza N_VERTICI
     * @throws GRBException
     */
    public static GRBVar[] creazione_u(GRBModel model) throws GRBException{

        GRBVar[] u = new GRBVar[N_VERTICI];
        for (int m=0; m<N_VERTICI; m++){
            u[m] = model.addVar(0.0, N_VERTICI, 0.0, GRB.INTEGER, "u "+ u);
        }
        return u;
    }

    /**
     * metodo che dato un modello gurobi crea al suo interno un vettore di variabili binarie z
     * @param model modello gurobi
     * @return vettore di variabili binarie z di grandezza 3
     * @throws GRBException
     */
    public static GRBVar[] creazione_z(GRBModel model) throws GRBException{

        GRBVar[] z = new GRBVar[3];
        for (int i = 0; i<3; i++){
            z[i] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "z " + z);
        }
        return z;
    }
}

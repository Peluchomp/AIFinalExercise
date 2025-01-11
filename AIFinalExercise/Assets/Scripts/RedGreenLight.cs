using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class RedGreenLight : MonoBehaviour
{
    public List<GameObject> gameObjects; 
    public string tag1 = "Tag1"; 
    public string tag2 = "Tag2"; 
    public float switchInterval = 2.0f; 
    public GameObject image;

    private bool useTag1 = true; 

    void Start()
    {
        image.GetComponent<Image>().color = Color.red;
        StartCoroutine(SwitchTagsPeriodically());
    }

    IEnumerator SwitchTagsPeriodically()
    {
        while (true)
        {
            yield return new WaitForSeconds(switchInterval);

            foreach (GameObject obj in gameObjects)
            {
                if (useTag1)
                {
                    obj.tag = tag1;
                    image.GetComponent<Image>().color = Color.red;
                }
                else
                {
                    obj.tag = tag2;
                    image.GetComponent<Image>().color = Color.green;
                }
            }

            useTag1 = !useTag1; // Alternar el indicador
        }
    }
}